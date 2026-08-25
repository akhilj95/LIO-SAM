#!/usr/bin/env python3
"""Offline converter: Bunker Mini packet bag -> a bag LIO-SAM can play directly.

The recorder captured raw UDP (`/hesai/lidar_packets`, hesai_ros_driver/msg/UdpFrame)
rather than PointCloud2, so the clouds have to be decoded before LIO-SAM can see
anything. Decoding is only reachable through the vendored driver node - the QT128
parsing (angle correction, firetimes, dual return, per-laser azimuth offsets) lives
in udp3_2_parser.h behind a ROS subscription - so this script drives the real driver
in-process rather than reimplementing the parser.

Two things about the driver's output make it unusable as-is, and this script fixes
both in the same pass:

1. THE CLOCK. The QT128 never had PTP/GPS lock, so its packets carry a free-running
   internal clock reading ~2020-10-09 while every other topic in the bag is on the
   recorder's 2026 clock. Nothing in the driver config can correct this: of the ~50
   keys it reads, `use_timestamp_type` is the only time-related one, and on the
   rosbag source setting it to 1 zeroes every timestamp (ReceivePacket() pushes
   packets with no recv_timestamp, and UdpPacket's constructor defaults it to 0).

   The fix is a constant offset. It is well determined because the recorded UdpFrame
   header stamp and the decoded cloud stamp are the SAME event on two clocks: both
   are `frame_start_timestamp`, the first packet of a revolution (hesai_lidar_sdk.hpp
   passes it to the packet callback, source_driver_ros2.hpp puts it on the cloud).
   The recorder ran with use_timestamp_type: 1, so its stamps are host receive times.
   Differencing them index-paired gives ~178268770.6583 s, constant to under a
   millisecond - far below the 2.5 ms IMU period.

2. THE TIME FIELD. The driver emits `timestamp` as FLOAT64 absolute seconds. LIO-SAM's
   velodyne path wants `time`, FLOAT32 relative to scan start, and its field scan in
   imageProjection.cpp only accepts the names "time" or "t". A cloud carrying
   "timestamp" silently disables deskewing.

Zero-return points are dropped by default. Roughly half of each frame is (0,0,0)
padding, and LIO-SAM discards it anyway via lidarMinRange, so keeping it only makes
the bag bigger and every later run slower.

/imu/data and /fix are copied straight from the source bag as raw serialized bytes,
interleaved in timestamp order. They never pass through the ROS graph, so they cannot
be dropped or re-timed.

Usage:
    ./convert_bunker.py ~/data/<input-bag-name> -o ~/data/<output-bag-name>
"""
from __future__ import annotations

import argparse
import os
import shutil
import signal
import subprocess
import sys
import time

import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.serialization import serialize_message
from sensor_msgs.msg import PointCloud2, PointField

import rosbag2_py

PACKET_TOPIC = '/hesai/lidar_packets'
# The driver's native output topic, set by ros_send_point_cloud_topic in its
# config.yaml. We subscribe to this.
CLOUD_TOPIC_IN = '/lidar_points'
# What we write. Deliberately distinct from the input name so a converted bag
# can never be mistaken for, or replayed alongside, the raw driver output.
CLOUD_TOPIC_OUT = '/lidar_points/corrected'
# Copied through untouched - the IMU needs no correction, since the clock fix
# is applied on the cloud side.
PASSTHROUGH = ('/imu/data', '/fix')

# The driver's layout (source_driver_ros2.hpp). Offsets are explicit because
# 'timestamp' sits at 18, which is not 8-byte aligned - numpy only accepts that
# through an explicit-offset dtype.
IN_DTYPE = np.dtype({
    'names': ['x', 'y', 'z', 'intensity', 'ring', 'timestamp'],
    'formats': ['<f4', '<f4', '<f4', '<f4', '<u2', '<f8'],
    'offsets': [0, 4, 8, 12, 16, 18],
    'itemsize': 26,
})

# What LIO-SAM's VelodynePointXYZIRT expects. pcl::fromROSMsg matches by field
# NAME and reads the offset from the message, so this need not mirror the C++
# struct's padding - only the names and types have to line up.
OUT_DTYPE = np.dtype({
    'names': ['x', 'y', 'z', 'intensity', 'ring', 'time'],
    'formats': ['<f4', '<f4', '<f4', '<f4', '<u2', '<f4'],
    'offsets': [0, 4, 8, 12, 16, 20],
    'itemsize': 24,
})

_F32 = PointField.FLOAT32
_U16 = PointField.UINT16
OUT_FIELDS = [
    PointField(name='x', offset=0, datatype=_F32, count=1),
    PointField(name='y', offset=4, datatype=_F32, count=1),
    PointField(name='z', offset=8, datatype=_F32, count=1),
    PointField(name='intensity', offset=12, datatype=_F32, count=1),
    PointField(name='ring', offset=16, datatype=_U16, count=1),
    PointField(name='time', offset=20, datatype=_F32, count=1),
]


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def stamp_to_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def sec_to_stamp(msg, seconds):
    """Write float seconds into msg.header.stamp, carrying the ns rounding."""
    sec = int(np.floor(seconds))
    nanosec = int(round((seconds - sec) * 1e9))
    if nanosec >= 1_000_000_000:      # rounding can push it over a whole second
        sec += 1
        nanosec -= 1_000_000_000
    msg.header.stamp.sec = sec
    msg.header.stamp.nanosec = nanosec


class Passthrough:
    """Streams /imu/data and /fix out of the source bag in timestamp order.

    Copying the serialized bytes verbatim keeps these topics byte-identical and,
    more importantly, immune to the drops a 400 Hz ROS round-trip could introduce.
    """

    def __init__(self, source_uri, writer, topics):
        self._reader = rosbag2_py.SequentialReader()
        self._reader.open(
            rosbag2_py.StorageOptions(uri=source_uri, storage_id='mcap'),
            rosbag2_py.ConverterOptions('', ''))
        wanted = [t for t in self._reader.get_all_topics_and_types()
                  if t.name in topics]
        if len(wanted) != len(topics):
            found = {t.name for t in wanted}
            raise SystemExit('source bag is missing {}'.format(
                sorted(set(topics) - found)))
        for i, meta in enumerate(wanted):
            writer.create_topic(rosbag2_py.TopicMetadata(
                id=i + 1, name=meta.name, type=meta.type,
                serialization_format='cdr'))
        self._writer = writer
        self._topics = set(topics)
        self._pending = None
        self.count = 0
        self._advance()

    def _advance(self):
        self._pending = None
        while self._reader.has_next():
            topic, data, ts = self._reader.read_next()
            if topic in self._topics:
                self._pending = (topic, data, ts)
                return

    def drain_until(self, ts_ns):
        while self._pending is not None and self._pending[2] <= ts_ns:
            self._writer.write(*self._pending)
            self.count += 1
            self._advance()

    def flush(self):
        while self._pending is not None:
            self._writer.write(*self._pending)
            self.count += 1
            self._advance()

    def stop(self):
        """Give up the rest of the source bag without writing it."""
        self._pending = None


class Converter(Node):
    def __init__(self, writer, passthrough, offset, calib_frames, drop_empty):
        super().__init__('bunker_converter')
        self._writer = writer
        self._passthrough = passthrough
        self._offset = offset
        self._calib_frames = calib_frames
        self._drop_empty = drop_empty

        self._packet_stamps = []
        self._buffered = []          # clouds held until the offset is known
        self.frames = 0
        self.points_in = 0
        self.points_out = 0
        self.last_cloud_walltime = None
        self._last_cloud_ts_ns = None
        self._start_wait = time.monotonic()
        self._layout_checked = False

        writer.create_topic(rosbag2_py.TopicMetadata(
            id=0, name=CLOUD_TOPIC_OUT, type='sensor_msgs/msg/PointCloud2',
            serialization_format='cdr'))

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=100)
        self._cloud_sub = self.create_subscription(
            PointCloud2, CLOUD_TOPIC_IN, self._on_cloud, qos)

        # Only needed to derive the offset. UdpFrame is ~6 MB per message, so the
        # subscription is torn down the moment calibration completes.
        self._packet_sub = None
        if self._offset is None:
            from hesai_ros_driver.msg import UdpFrame
            self._packet_sub = self.create_subscription(
                UdpFrame, PACKET_TOPIC, self._on_packet,
                QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                           history=HistoryPolicy.KEEP_LAST, depth=50))

    def _on_packet(self, msg):
        if self._offset is None:
            self._packet_stamps.append(stamp_to_sec(msg.header.stamp))
            self._try_calibrate()

    def _try_calibrate(self):
        n = min(len(self._packet_stamps), len(self._buffered))
        if n < self._calib_frames:
            return
        # Index pairing: both the driver and this node start before playback, so
        # each stream's k-th message is the k-th revolution. The median absorbs
        # the odd dropped UdpFrame, which would otherwise show up as one pair
        # displaced by a whole 100 ms scan period.
        diffs = np.array([self._packet_stamps[i] - self._buffered[i][0]
                          for i in range(n)])
        self._offset = float(np.median(diffs))
        spread = float(diffs.max() - diffs.min())
        self.get_logger().info(
            'clock offset = {:.4f} s  (median of {} frames, spread {:.2f} ms)'
            .format(self._offset, n, spread * 1e3))
        if spread > 0.05:
            self.get_logger().warn(
                'offset spread is {:.1f} ms - frames may have been dropped '
                'during calibration; verify with --offset if the map smears'
                .format(spread * 1e3))
        if self._packet_sub is not None:
            self.destroy_subscription(self._packet_sub)
            self._packet_sub = None
        for _, msg in self._buffered:
            self._emit(msg)
        self._buffered.clear()

    def _check_layout(self, msg):
        got = [(f.name, f.datatype, f.offset) for f in msg.fields]
        want = [('x', _F32, 0), ('y', _F32, 4), ('z', _F32, 8),
                ('intensity', _F32, 12), ('ring', _U16, 16),
                ('timestamp', PointField.FLOAT64, 18)]
        if got != want or msg.point_step != IN_DTYPE.itemsize:
            raise SystemExit(
                'unexpected cloud layout from the driver.\n  got  {} step={}\n'
                '  want {} step={}\nThe driver version may have changed; update '
                'IN_DTYPE.'.format(got, msg.point_step, want, IN_DTYPE.itemsize))
        self._layout_checked = True

    def _on_cloud(self, msg):
        self.last_cloud_walltime = time.monotonic()
        if not self._layout_checked:
            self._check_layout(msg)
        if self._offset is None:
            self._buffered.append((stamp_to_sec(msg.header.stamp), msg))
            self._try_calibrate()
            return
        self._emit(msg)

    def _emit(self, msg):
        n = msg.width * msg.height
        raw = np.frombuffer(bytes(msg.data), dtype=IN_DTYPE, count=n)
        self.points_in += n

        keep = (np.isfinite(raw['x']) & np.isfinite(raw['y'])
                & np.isfinite(raw['z']))
        if self._drop_empty:
            # No-return points come back as exact (0,0,0), not NaN.
            keep &= ~((raw['x'] == 0.0) & (raw['y'] == 0.0) & (raw['z'] == 0.0))
        sel = raw[keep]

        # t0 is the frame start on the lidar's own clock. Taking it from the
        # header rather than sel['timestamp'].min() keeps 'time' referenced to
        # the same instant the corrected stamp names, even after filtering.
        t0 = stamp_to_sec(msg.header.stamp)

        out = np.empty(sel.size, dtype=OUT_DTYPE)
        for name in ('x', 'y', 'z', 'intensity', 'ring'):
            out[name] = sel[name]
        out['time'] = (sel['timestamp'] - t0).astype('<f4')

        fixed = PointCloud2()
        sec_to_stamp(fixed, t0 + self._offset)
        fixed.header.frame_id = msg.header.frame_id
        fixed.height = 1
        fixed.width = int(sel.size)
        fixed.fields = OUT_FIELDS
        fixed.is_bigendian = False
        fixed.point_step = OUT_DTYPE.itemsize
        fixed.row_step = OUT_DTYPE.itemsize * int(sel.size)
        fixed.data = out.tobytes()
        fixed.is_dense = True

        ts_ns = stamp_to_ns(fixed.header.stamp)
        self._passthrough.drain_until(ts_ns)
        self._writer.write(CLOUD_TOPIC_OUT, serialize_message(fixed), ts_ns)
        self._last_cloud_ts_ns = ts_ns

        self.frames += 1
        self.points_out += int(sel.size)
        if self.frames % 200 == 0:
            self.get_logger().info('{} frames written'.format(self.frames))

    def finish(self):
        """Emit anything still held back, then close out the passthrough tail."""
        if self._offset is None and self._buffered:
            self._try_calibrate()
        if self._offset is None and self._buffered:
            raise SystemExit(
                'never saw enough {} messages to derive the clock offset. '
                'Pass --offset explicitly.'.format(PACKET_TOPIC))
        # Carry IMU/GPS one scan period past the last cloud and no further.
        # Samples beyond that are useless to LIO-SAM, and without the cap a
        # --duration slice would still drag in the whole source stream.
        if self._last_cloud_ts_ns is None:
            self._passthrough.stop()
        else:
            self._passthrough.drain_until(self._last_cloud_ts_ns + 100_000_000)
            self._passthrough.stop()


def driver_processes():
    """PIDs of any hesai driver node already running.

    Resolved through /proc/<pid>/exe rather than a command-line match: pgrep -f
    also matches any shell whose command line merely mentions the driver path
    (including the one that launched this script), and pgrep -x cannot help
    because 'hesai_ros_driver_node' exceeds the 15-character comm limit.
    """
    pids = []
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        try:
            exe = os.readlink(os.path.join('/proc', entry, 'exe'))
        except OSError:
            continue          # gone, or not ours to inspect
        if os.path.basename(exe) == 'hesai_ros_driver_node':
            pids.append(entry)
    return pids


def preflight(hesai_config, packet_topic):
    """The driver reads this file at runtime from its SOURCE tree (PROJECT_PATH is
    baked in at compile time and RUN_IN_ROS_WORKSPACE is ROS 1 only), so a stale
    setting here silently produces an empty run."""
    if not os.path.exists(hesai_config):
        raise SystemExit('hesai config not found: {}'.format(hesai_config))
    with open(hesai_config) as fh:
        cfg = yaml.safe_load(fh)
    drv = cfg['lidar'][0]['driver']
    ros = cfg['lidar'][0]['ros']
    problems = []
    if drv.get('source_type') != 3:
        problems.append('source_type is {}, expected 3 (packet rosbag)'
                        .format(drv.get('source_type')))
    if ros.get('ros_recv_packet_topic') != packet_topic:
        problems.append('ros_recv_packet_topic is {!r}, expected {!r}'
                        .format(ros.get('ros_recv_packet_topic'), packet_topic))
    if drv.get('use_timestamp_type') != 0:
        problems.append(
            'use_timestamp_type is {}, expected 0 - type 1 zeroes every '
            'timestamp on the rosbag source'.format(drv.get('use_timestamp_type')))
    for key in ('correction_file_path', 'firetimes_path'):
        path = drv.get('rosbag_type', {}).get(key, '')
        if not os.path.exists(path):
            problems.append('rosbag_type.{} does not exist: {!r}'.format(key, path))
    if problems:
        raise SystemExit('hesai config ({}) is not set up for this conversion:\n  '
                         .format(hesai_config) + '\n  '.join(problems))


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    default_cfg = os.path.normpath(os.path.join(
        here, '..', '..', '..', 'HesaiLidar_ROS_2.0', 'config', 'config.yaml'))

    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('input', help='source bag directory (with metadata.yaml)')
    ap.add_argument('-o', '--output', required=True, help='output bag directory')
    ap.add_argument('--offset', type=float, default=None,
                    help='lidar->bag clock offset in seconds. Derived '
                         'automatically when omitted.')
    ap.add_argument('--calib-frames', type=int, default=15,
                    help='frames used to derive the offset (default: 15)')
    ap.add_argument('--rate', type=float, default=1.0,
                    help='bag play rate. Above ~1.0 the driver starts dropping '
                         'packets, which corrupts frames.')
    ap.add_argument('--keep-empty', action='store_true',
                    help='keep zero-return (0,0,0) points')
    ap.add_argument('--duration', type=float, default=None,
                    help='convert only the first N seconds (for a quick check)')
    ap.add_argument('--hesai-config', default=default_cfg)
    ap.add_argument('--drain-timeout', type=float, default=5.0,
                    help='seconds without a cloud after playback ends before '
                         'closing the bag')
    ap.add_argument('--force', action='store_true',
                    help='overwrite the output bag if it exists')
    args = ap.parse_args()

    if not os.path.exists(os.path.join(args.input, 'metadata.yaml')):
        raise SystemExit('not a bag directory: {}'.format(args.input))
    if os.path.exists(args.output):
        if not args.force:
            raise SystemExit('output exists: {} (use --force)'.format(args.output))
        shutil.rmtree(args.output)

    preflight(args.hesai_config, PACKET_TOPIC)

    # A driver left over from an earlier run publishes on CLOUD_TOPIC_IN too. The
    # result is not an error anywhere - just double the frames, interleaved from
    # two decoders, which breaks the calibration pairing and corrupts the output.
    running = driver_processes()
    if running:
        raise SystemExit(
            'a hesai driver is already running (pids {}). Stop it first:\n'
            '    pkill -f lib/hesai_ros_driver/'.format(', '.join(running)))

    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=args.output, storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))

    driver = None
    play = None
    rclpy.init()
    try:
        passthrough = Passthrough(args.input, writer, PASSTHROUGH)
        node = Converter(writer, passthrough, args.offset,
                         args.calib_frames, not args.keep_empty)

        driver_log = open(os.path.join(os.path.dirname(args.output) or '.',
                                       'hesai_driver.log'), 'w')
        # start_new_session so the driver gets its own process group: `ros2 run`
        # is a wrapper that does NOT forward SIGINT to the node it spawns, so
        # signalling the wrapper alone leaves the node running. A survivor
        # publishing on CLOUD_TOPIC_IN alongside the next run's driver silently
        # doubles the frame rate and destroys the calibration pairing.
        driver = subprocess.Popen(
            ['ros2', 'run', 'hesai_ros_driver', 'hesai_ros_driver_node'],
            stdout=driver_log, stderr=subprocess.STDOUT, start_new_session=True)

        # Both subscribers must be up before the first packet, or the index
        # pairing the offset relies on would be skewed.
        node.get_logger().info('waiting for the driver to come up...')
        time.sleep(5.0)
        if driver.poll() is not None:
            raise SystemExit('driver exited immediately; see hesai_driver.log')

        play_cmd = ['ros2', 'bag', 'play', args.input, '--topics', PACKET_TOPIC,
                    '--rate', str(args.rate)]
        if args.duration is not None:
            play_cmd += ['--playback-duration', str(args.duration)]
        play = subprocess.Popen(play_cmd, stdout=subprocess.DEVNULL,
                                stderr=subprocess.STDOUT, start_new_session=True)
        node.get_logger().info('playing {} ...'.format(args.input))

        while play.poll() is None:
            rclpy.spin_once(node, timeout_sec=0.1)
        # Playback is done, but frames are still working through the driver's
        # parser threads and our queue.
        node.get_logger().info('playback finished, draining...')
        while True:
            rclpy.spin_once(node, timeout_sec=0.1)
            last = node.last_cloud_walltime
            if last is not None and time.monotonic() - last > args.drain_timeout:
                break
            if last is None and time.monotonic() - node._start_wait > 60:
                break

        node.finish()
        print('\nframes written : {}'.format(node.frames))
        print('points         : {} -> {}  ({:.0f}% kept)'.format(
            node.points_in, node.points_out,
            100.0 * node.points_out / max(node.points_in, 1)))
        print('passthrough    : {} messages from {}'.format(
            passthrough.count, ', '.join(PASSTHROUGH)))
        print('offset applied : {:.4f} s'.format(node._offset))
        print('output         : {}'.format(args.output))
    finally:
        for proc in (play, driver):
            if proc is None or proc.poll() is not None:
                continue
            try:
                pgid = os.getpgid(proc.pid)
            except ProcessLookupError:
                continue
            os.killpg(pgid, signal.SIGINT)
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(pgid, signal.SIGKILL)
                proc.wait(timeout=5)
        leftover = driver_processes()
        if leftover:
            print('WARNING: hesai driver still running (pids {}); kill it before '
                  'the next run or the frame rate will double'
                  .format(', '.join(leftover)), file=sys.stderr)
        try:
            writer.close()
        except Exception:
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
