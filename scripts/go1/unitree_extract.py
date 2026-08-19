#!/usr/bin/env python3
"""Stage 1: extract LIO-SAM inputs from raw Unitree/Go1 ROS 1 bags.

Port of Forestry-Robotics-UC/fruc_lio_sam scripts/unitree_to_lio_offline_rtk.py.
Reads ROS 1 bags and writes one ROS 2 bag, doing only topic renaming and the
Unitree HighState -> sensor_msgs/Imu conversion. Clouds and GPS pass through
untouched; go1_preprocess.py (stage 2) does the filtering and IMU resampling.

    ./unitree_extract.py raw_0.bag raw_1.bag -o extracted

Several input bags are read in order into a single output, since the Unitree
recordings are chunks of one run. Run it once per bag if you want them separate.

No ROS 1 installation is needed: ROS 1 bags embed their message definitions, so
rosbags registers unitree_legged_msgs/HighState straight from the file.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader

import bagio

# ROS 1 topic -> ROS 2 topic. Everything here except /high_state is copied
# through with only its message type rebuilt.
TOPIC_MAP = {
    '/rslidar_points': '/rslidar_points',
    '/camera/imu': '/camera/imu',
    '/reach/fix': '/gps/fix',
    '/reach/vel': '/gps/vel',
    '/reach/time_ref': '/gps/time_ref',
}

HIGH_STATE_TOPIC = '/high_state'
IMU_OUT_TOPIC = '/imu_data'


def high_state_to_imu(msg, stamp, frame_id, quat_order):
    """Unitree HighState -> sensor_msgs/Imu.

    HighState carries no covariances, so they stay zero: LIO-SAM never reads
    them, and robot_localization is configured per-axis in params.yaml instead.
    """
    types = bagio.TYPESTORE.types
    imu = msg.imu
    q = list(imu.quaternion)
    if quat_order == 'wxyz':
        w, x, y, z = q
    else:
        x, y, z, w = q

    zeros = np.zeros(9, dtype=np.float64)
    return types['sensor_msgs/msg/Imu'](
        header=types['std_msgs/msg/Header'](stamp=stamp, frame_id=frame_id),
        orientation=types['geometry_msgs/msg/Quaternion'](x=float(x), y=float(y),
                                                          z=float(z), w=float(w)),
        orientation_covariance=zeros.copy(),
        angular_velocity=types['geometry_msgs/msg/Vector3'](
            x=float(imu.gyroscope[0]), y=float(imu.gyroscope[1]), z=float(imu.gyroscope[2])),
        angular_velocity_covariance=zeros.copy(),
        linear_acceleration=types['geometry_msgs/msg/Vector3'](
            x=float(imu.accelerometer[0]), y=float(imu.accelerometer[1]),
            z=float(imu.accelerometer[2])),
        linear_acceleration_covariance=zeros.copy(),
    )


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('inputs', nargs='+', type=Path, help='ROS 1 .bag files, in order')
    ap.add_argument('-o', '--output', type=Path, required=True,
                    help='output ROS 2 bag directory (must not exist)')
    ap.add_argument('--imu-frame', default='base_link',
                    help='frame_id stamped on the converted IMU (default: base_link). '
                         'FRUC used "base", which none of the URDFs declare.')
    ap.add_argument('--quat-order', choices=['xyzw', 'wxyz'], default='xyzw',
                    help='element order of HighState.imu.quaternion. Default xyzw '
                         'reproduces FRUC; the Unitree SDK documents wxyz, so try '
                         'that if the initial heading looks rotated.')
    args = ap.parse_args()

    for path in args.inputs:
        if not path.exists():
            sys.exit(f'input bag not found: {path}')
    if args.output.exists():
        sys.exit(f'output already exists, refusing to overwrite: {args.output}')

    with bagio.Bag2Writer(args.output) as writer:
        for path in args.inputs:
            print(f'reading {path}')
            with AnyReader([path]) as reader:
                wanted = [c for c in reader.connections
                          if c.topic in TOPIC_MAP or c.topic == HIGH_STATE_TOPIC]
                if not wanted:
                    print('  no matching topics, skipped')
                    continue
                for conn, timestamp, raw in bagio.progress(
                        reader.messages(connections=wanted), unit=' msg'):
                    msg = reader.deserialize(raw, conn.msgtype)
                    if conn.topic == HIGH_STATE_TOPIC:
                        # HighState has no header; FRUC stamps it with the bag
                        # receive time, so keep doing that.
                        imu = high_state_to_imu(msg, bagio.ns_to_stamp(timestamp),
                                                args.imu_frame, args.quat_order)
                        writer.write(IMU_OUT_TOPIC, imu, timestamp)
                    else:
                        writer.write(TOPIC_MAP[conn.topic], bagio.to_ros2(msg), timestamp)

        print(f'\nwrote {args.output}')
        writer.report()


if __name__ == '__main__':
    main()
