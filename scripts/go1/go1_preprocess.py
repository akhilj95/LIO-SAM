#!/usr/bin/env python3
"""Stage 2: filter the cloud and resample the IMU for LIO-SAM.

Port of Forestry-Robotics-UC/fruc_lio_sam scripts/go1_lio_preprocessor.py.
Consumes the ROS 2 bag written by unitree_extract.py and produces the topics
params_rslidar.yaml expects:

    /rslidar_points        ->  /rslidar_points_dense   (finite, range-gated,
                                                        ring/time kept, is_dense)
    /imu_data              ->  /imu_data_resampled     (100 Hz, de-biased, EMA)
    everything else                                     passed through

Frames are renamed to match the URDF (rslidar -> velodyne, base -> base_link,
gps -> navsat_link), which is what lets navsat_transform_node find the antenna.

    ./go1_preprocess.py extracted -o preprocessed
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader

import bagio

# ---- LiDAR filtering ----
RANGE_MIN, RANGE_MAX = 0.5, 100.0
CLOUD_IN_TOPIC, CLOUD_OUT_TOPIC = '/rslidar_points', '/rslidar_points_dense'
OUT_POINT_STEP = 24
OUT_DTYPE = np.dtype({
    'names': ['x', 'y', 'z', 'intensity', 'ring', 'time'],
    'formats': ['f4', 'f4', 'f4', 'f4', 'u2', 'f4'],
    'offsets': [0, 4, 8, 12, 16, 20],
    'itemsize': OUT_POINT_STEP,
})

# ---- IMU resampling (values from FRUC) ----
IMU_IN_TOPIC, IMU_OUT_TOPIC = '/imu_data', '/imu_data_resampled'
PUB_RATE = 100.0
MAX_GAP = 0.08
ACC_MAX, ACC_MIN = 25.0, 5.0
GYR_MAX = 6.0
EMA_ALPHA = 0.3
GRAVITY_N = 9.8
BIAS_INIT = 200
KEEP_FIRST_N = 200

FRAME_MAP = {'rslidar': 'velodyne', 'base': 'base_link', 'gps': 'navsat_link'}


def rename_frame(header):
    header.frame_id = FRAME_MAP.get(header.frame_id, header.frame_id)
    return header


def out_fields():
    PointField = bagio.TYPESTORE.types['sensor_msgs/msg/PointField']
    spec = [('x', 0, 7), ('y', 4, 7), ('z', 8, 7), ('intensity', 12, 7),
            ('ring', 16, 4), ('time', 20, 7)]     # 7 = FLOAT32, 4 = UINT16
    return [PointField(name=n, offset=o, datatype=d, count=1) for n, o, d in spec]


def filter_cloud(msg, fields):
    """Drop non-finite and out-of-range points, keeping ring/time."""
    arr = bagio.cloud_to_array(msg)
    names = arr.dtype.names

    x, y, z = (arr['x'].astype(np.float32), arr['y'].astype(np.float32),
               arr['z'].astype(np.float32))
    intensity = (arr['intensity'].astype(np.float32) if 'intensity' in names
                 else np.zeros(len(arr), np.float32))

    keep = np.isfinite(x) & np.isfinite(y) & np.isfinite(z) & np.isfinite(intensity)
    rng = np.sqrt(x.astype(np.float64) ** 2 + y ** 2 + z ** 2)
    keep &= (rng >= RANGE_MIN) & (rng <= RANGE_MAX)

    out = np.zeros(int(keep.sum()), dtype=OUT_DTYPE)
    out['x'], out['y'], out['z'] = x[keep], y[keep], z[keep]
    out['intensity'] = intensity[keep]
    out['ring'] = arr['ring'][keep] if 'ring' in names else 0
    out['time'] = arr['time'][keep] if 'time' in names else 0.0

    return bagio.array_to_cloud(out, rename_frame(msg.header), fields, OUT_POINT_STEP)


def slerp(q1, q2, u):
    q1, q2 = np.asarray(q1, float), np.asarray(q2, float)
    dot = float(np.dot(q1, q2))
    if dot < 0.0:
        q2, dot = -q2, -dot
    if dot > 0.9995:
        r = q1 + u * (q2 - q1)
        return r / np.linalg.norm(r)
    theta_0 = math.acos(dot)
    sin_0 = math.sin(theta_0)
    theta = theta_0 * u
    s0 = math.cos(theta) - dot * math.sin(theta) / sin_0
    s1 = math.sin(theta) / sin_0
    return s0 * q1 + s1 * q2


def _vec(msg_obj):
    return np.array([msg_obj.x, msg_obj.y, msg_obj.z], dtype=float)


def _quat(msg_obj):
    return np.array([msg_obj.x, msg_obj.y, msg_obj.z, msg_obj.w], dtype=float)


def build_imu(stamp, frame_id, quat, gyr, acc, template):
    types = bagio.TYPESTORE.types
    return types['sensor_msgs/msg/Imu'](
        header=types['std_msgs/msg/Header'](stamp=stamp, frame_id=frame_id),
        orientation=types['geometry_msgs/msg/Quaternion'](
            x=float(quat[0]), y=float(quat[1]), z=float(quat[2]), w=float(quat[3])),
        orientation_covariance=template.orientation_covariance.copy(),
        angular_velocity=types['geometry_msgs/msg/Vector3'](
            x=float(gyr[0]), y=float(gyr[1]), z=float(gyr[2])),
        angular_velocity_covariance=template.angular_velocity_covariance.copy(),
        linear_acceleration=types['geometry_msgs/msg/Vector3'](
            x=float(acc[0]), y=float(acc[1]), z=float(acc[2])),
        linear_acceleration_covariance=template.linear_acceleration_covariance.copy(),
    )


def resample_imu(samples, frame_id):
    """Interpolate to PUB_RATE, remove bias, smooth, keep timestamps monotonic.

    The first KEEP_FIRST_N messages are emitted raw so LIO-SAM's preintegration
    starts from untouched data.

    Two deliberate departures from FRUC's version, both bugs there:
      * its resampling loop restarted at t0 after already emitting the raw
        block, then shifted any colliding stamp forward - so early samples were
        interpolated at one time and stamped with another.
      * it re-scanned the whole sample list per output tick, which is O(n^2);
        searchsorted makes it O(n log n).
    """
    if len(samples) < 3:
        return samples

    samples.sort(key=lambda s: s[0])
    times = np.array([t for t, _ in samples])
    msgs = [m for _, m in samples]

    head = np.array([np.concatenate([_vec(m.linear_acceleration),
                                     _vec(m.angular_velocity)]) for m in msgs[:BIAS_INIT]])
    acc_bias = head[:, :3].mean(axis=0) - np.array([0.0, 0.0, GRAVITY_N])
    gyr_bias = head[:, 3:].mean(axis=0)

    out = []
    keep_n = min(KEEP_FIRST_N, len(msgs))
    for t, m in samples[:keep_n]:
        m.header.stamp = bagio.ns_to_stamp(round(t, 6) * 1e9)
        m.header.frame_id = frame_id
        out.append((t, m))

    last_acc = np.array([0.0, 0.0, GRAVITY_N])
    last_gyr = np.zeros(3)
    t = round(times[keep_n - 1] + 1.0 / PUB_RATE, 6)
    t_end = times[-1]

    while t <= t_end:
        i = int(np.searchsorted(times, t, side='right')) - 1
        i = min(max(i, 0), len(times) - 2)
        ta, tb = times[i], times[i + 1]
        a, b = msgs[i], msgs[i + 1]

        if tb - ta <= MAX_GAP and tb > ta:
            u = min(1.0, max(0.0, (t - ta) / (tb - ta)))
            acc = _vec(a.linear_acceleration) + u * (_vec(b.linear_acceleration)
                                                     - _vec(a.linear_acceleration))
            gyr = _vec(a.angular_velocity) + u * (_vec(b.angular_velocity)
                                                  - _vec(a.angular_velocity))
            quat = slerp(_quat(a.orientation), _quat(b.orientation), u)

            acc, gyr = acc - acc_bias, gyr - gyr_bias
            if np.linalg.norm(acc) > ACC_MAX or np.linalg.norm(acc) < ACC_MIN \
                    or np.linalg.norm(gyr) > GYR_MAX:
                acc, gyr = last_acc, last_gyr
            else:
                acc = EMA_ALPHA * acc + (1 - EMA_ALPHA) * last_acc
                gyr = EMA_ALPHA * gyr + (1 - EMA_ALPHA) * last_gyr
                last_acc, last_gyr = acc, gyr
        else:
            # Gap too wide to interpolate across: hold the last good sample.
            acc, gyr, quat = last_acc, last_gyr, _quat(a.orientation)

        out.append((t, build_imu(bagio.ns_to_stamp(round(t, 6) * 1e9), frame_id,
                                 quat, gyr, acc, msgs[0])))
        t = round(t + 1.0 / PUB_RATE, 6)

    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('input', type=Path, help='ROS 2 bag from unitree_extract.py')
    ap.add_argument('-o', '--output', type=Path, required=True,
                    help='output ROS 2 bag directory (must not exist)')
    ap.add_argument('--imu-frame', default='base_link',
                    help='frame_id stamped on the resampled IMU (default: base_link)')
    ap.add_argument('--keep-raw-cloud', action='store_true',
                    help='also copy the unfiltered /rslidar_points through, as FRUC did')
    args = ap.parse_args()

    if not args.input.exists():
        sys.exit(f'input bag not found: {args.input}')
    if args.output.exists():
        sys.exit(f'output already exists, refusing to overwrite: {args.output}')

    fields = out_fields()
    imu_samples = []

    with bagio.Bag2Writer(args.output) as writer:
        with AnyReader([args.input]) as reader:
            for conn, timestamp, raw in bagio.progress(reader.messages(), unit=' msg'):
                msg = reader.deserialize(raw, conn.msgtype)

                if conn.topic == CLOUD_IN_TOPIC:
                    writer.write(CLOUD_OUT_TOPIC, filter_cloud(msg, fields), timestamp)
                    if args.keep_raw_cloud:
                        writer.write(conn.topic, msg, timestamp)
                elif conn.topic == IMU_IN_TOPIC:
                    # Keyed on the bag timestamp, not header.stamp, so the output
                    # keeps one coherent clock even if a source bag's headers and
                    # receive times disagree. The resampled headers are restamped
                    # from it below. This is what FRUC did.
                    imu_samples.append((timestamp / 1e9, msg))
                else:
                    if hasattr(msg, 'header') and hasattr(msg.header, 'frame_id'):
                        rename_frame(msg.header)
                    writer.write(conn.topic, msg, timestamp)

        print(f'\nresampling {len(imu_samples)} IMU samples to {PUB_RATE:.0f} Hz ...')
        for t, msg in resample_imu(imu_samples, args.imu_frame):
            writer.write(IMU_OUT_TOPIC, msg, int(round(t * 1e9)))

        print(f'\nwrote {args.output}')
        writer.report()


if __name__ == '__main__':
    main()
