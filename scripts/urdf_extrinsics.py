#!/usr/bin/env python3
"""Derive LIO-SAM's extrinsic block from a robot description.

All three extrinsics LIO-SAM reads are T_lb (lidar <- imu); see the CONVENTION
comment in config/params_bunkermini.yaml, which was read off the code:

  extrinsicRot    R_lidar<-imu. utility.hpp:321 rotates IMU-frame acc/gyr into
                  lidar axes with it.
  extrinsicRPY    R_lidar<-imu as well. utility.hpp:254 inverts it and :333
                  right-multiplies the IMU's q_wb, giving q_wl = q_wb * q_bl.
                  Equal to extrinsicRot whenever the IMU reports inertial and
                  attitude data in the same body frame.
  extrinsicTrans  The IMU ORIGIN EXPRESSED IN LIDAR AXES - imuPreintegration
                  .cpp:226 pairs it with an identity rotation, because
                  imuConverter has already rotated the data.

So the whole job is composing the URDF chain both links hang off of:

    T_lb = T_root<-lidar^-1 * T_root<-imu

Usage (inside the container, with ROS sourced - xacro and urdf_parser_py are
both ROS python modules):

    ./urdf_extrinsics.py ../config/fruc_bunkermini.urdf.xacro \\
        --lidar-link hesai_lidar --imu-link imu

The IMU link cannot be inferred: params files carry lidarFrame but no imuFrame,
since no node ever needs one. Both links are therefore explicit arguments.
"""

import argparse
import sys
import xml.etree.ElementTree as ET

import numpy as np


def rpy_to_matrix(roll, pitch, yaw):
    """URDF convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [    -sp,                cp * sr,                cp * cr],
    ])


def matrix_to_rpy(R):
    """Inverse of rpy_to_matrix, for the human-readable summary only."""
    pitch = np.arctan2(-R[2, 0], np.hypot(R[0, 0], R[1, 0]))
    if np.isclose(np.cos(pitch), 0.0, atol=1e-9):  # gimbal lock
        return np.arctan2(-R[1, 2], R[1, 1]), pitch, 0.0
    return (np.arctan2(R[2, 1], R[2, 2]), pitch, np.arctan2(R[1, 0], R[0, 0]))


def load_urdf(path, mappings):
    """Return URDF XML text, running the file through xacro when needed."""
    if path.endswith('.xacro'):
        import xacro
        doc = xacro.process_file(path, mappings=mappings)
        return doc.toprettyxml(indent='  ')
    with open(path) as f:
        return f.read()


def strip_cosmetics(xml_text):
    """Drop <visual>/<collision>/<inertial>, which we do not read.

    urdf_parser_py is strict about required attributes, and the SolidWorks
    exporter these descriptions came out of emits e.g. a bare <texture/> inside
    a material (bunkermini_sensors.urdf.xacro, link 'sensor_link') - enough to
    abort the whole parse. None of it bears on the joint tree, so it goes.
    """
    root = ET.fromstring(xml_text)
    for link in root.findall('link'):
        for tag in ('visual', 'collision', 'inertial'):
            for element in link.findall(tag):
                link.remove(element)
    return ET.tostring(root, encoding='unicode')


def chain_to_root(robot, link):
    """Walk parent joints from `link` up to the root, nearest joint first."""
    if link not in robot.link_map:
        sys.exit(f"link '{link}' is not in the description. Links: "
                 + ', '.join(sorted(robot.link_map)))
    chain, current, seen = [], link, {link}
    while current in robot.parent_map:
        joint_name, parent = robot.parent_map[current]
        joint = robot.joint_map[joint_name]
        if joint.type != 'fixed':
            print(f"warning: joint '{joint_name}' ({joint.type}) lies between "
                  f"'{link}' and the root; using its zero configuration. A "
                  f"non-fixed joint means the lidar<-imu transform is not "
                  f"actually constant.", file=sys.stderr)
        chain.append(joint)
        if parent in seen:
            sys.exit(f"cycle in the joint tree at '{parent}'")
        seen.add(parent)
        current = parent
    return chain, current


def transform_from_root(robot, link):
    """T_root<-link, as a 4x4 homogeneous matrix."""
    chain, root = chain_to_root(robot, link)
    T = np.eye(4)
    for joint in reversed(chain):  # root-most joint applied first
        origin = joint.origin
        xyz = origin.xyz if origin is not None and origin.xyz else [0.0] * 3
        rpy = origin.rpy if origin is not None and origin.rpy else [0.0] * 3
        step = np.eye(4)
        step[:3, :3] = rpy_to_matrix(*rpy)
        step[:3, 3] = xyz
        T = T @ step
    return T, root, [j.name for j in reversed(chain)]


def snap_to_axis_aligned(R, tol_deg):
    """Round R to the nearest signed axis permutation, if it is within tol.

    SolidWorks-exported chains accumulate rounding: Bunker Mini's lidar and IMU
    are both mounted at a nominal Rz(+90) off base_link, but composing the
    exported joints leaves 0.18 deg of residual. Mounts machined to axis
    alignment are better represented by the exact matrix - that is the value
    the hand-derived params files carry. Returns (R_snapped, residual_deg), or
    (R, None) when no axis-aligned matrix is within tolerance.
    """
    candidate = np.zeros((3, 3))
    for col in range(3):
        row = int(np.argmax(np.abs(R[:, col])))
        candidate[row, col] = np.sign(R[row, col])
    if not np.isclose(abs(np.linalg.det(candidate)), 1.0):
        return R, None  # not a permutation: two columns claimed the same axis
    # Geodesic distance between the two rotations.
    cos_angle = (np.trace(candidate.T @ R) - 1.0) / 2.0
    residual = np.degrees(np.arccos(np.clip(cos_angle, -1.0, 1.0)))
    if residual > tol_deg:
        return R, None
    return candidate, residual


def fmt(x, precision):
    """Snap values that are a hair off an exact 0/±1 before printing."""
    for exact in (0.0, 1.0, -1.0):
        if abs(x - exact) < 1e-9:
            x = exact
    return f'{x:.{precision}f}'


def main():
    p = argparse.ArgumentParser(
        description='Compute LIO-SAM extrinsics (T_lb) from a URDF/xacro.')
    p.add_argument('urdf', help='path to a .urdf or .urdf.xacro file')
    p.add_argument('--lidar-link', required=True,
                   help="the lidar frame, i.e. params' lidarFrame")
    p.add_argument('--imu-link', required=True,
                   help='the link the IMU driver stamps its messages with')
    p.add_argument('--mappings', nargs='*', default=[], metavar='NAME:=VALUE',
                   help='xacro arguments, if the description takes any')
    p.add_argument('--precision', type=int, default=6,
                   help='decimal places in the emitted values (default: 6)')
    p.add_argument('--snap-deg', type=float, default=0.0, metavar='DEG',
                   help='snap the rotation to the nearest exact axis '
                        'permutation when it is within DEG of one, as the '
                        'hand-derived params files do (default: 0, off). The '
                        'residual is reported in the emitted comment.')
    args = p.parse_args()

    mappings = {}
    for m in args.mappings:
        if ':=' not in m:
            sys.exit(f"malformed mapping '{m}', expected NAME:=VALUE")
        name, _, value = m.partition(':=')
        mappings[name] = value

    from urdf_parser_py.urdf import URDF
    robot = URDF.from_xml_string(strip_cosmetics(load_urdf(args.urdf, mappings)))

    T_root_lidar, root, lidar_chain = transform_from_root(robot, args.lidar_link)
    T_root_imu, imu_root, imu_chain = transform_from_root(robot, args.imu_link)
    if root != imu_root:
        sys.exit(f"'{args.lidar_link}' and '{args.imu_link}' are in "
                 f"disconnected trees (roots '{root}' and '{imu_root}'); no "
                 f"transform between them exists.")

    # T_lb = T_root<-lidar^-1 * T_root<-imu, inverted in closed form.
    R_rl, t_rl = T_root_lidar[:3, :3], T_root_lidar[:3, 3]
    T_lidar_root = np.eye(4)
    T_lidar_root[:3, :3] = R_rl.T
    T_lidar_root[:3, 3] = -R_rl.T @ t_rl
    T_lb = T_lidar_root @ T_root_imu

    R, t = T_lb[:3, :3], T_lb[:3, 3]
    snapped = None
    if args.snap_deg > 0.0:
        R, snapped = snap_to_axis_aligned(R, args.snap_deg)
    roll, pitch, yaw = np.degrees(matrix_to_rpy(R))
    n = args.precision

    print(f'# Derived from {args.urdf}')
    print(f'#   root frame:  {root}')
    print(f'#   {root} -> {args.lidar_link}: {" -> ".join(lidar_chain) or "(root itself)"}')
    print(f'#   {root} -> {args.imu_link}: {" -> ".join(imu_chain) or "(root itself)"}')
    print(f'#   R_lidar<-imu as RPY: '
          f'[{roll:.3f}, {pitch:.3f}, {yaw:.3f}] deg')
    if snapped is not None:
        print(f'#   rotation snapped to exact axis alignment, discarding '
              f'{snapped:.3f} deg of chain rounding')
    elif args.snap_deg > 0.0:
        print(f'#   NOT snapped: no axis-aligned rotation within '
              f'{args.snap_deg} deg. The mount is genuinely off-axis, or a '
              f'joint origin is wrong.')
    print('#')
    print('# All three are T_lb (lidar <- imu). extrinsicRPY is emitted equal to')
    print('# extrinsicRot, which holds when the IMU reports inertial and attitude')
    print('# data in the same body frame. If yours does not, override it by hand.')
    print()
    print('    extrinsicTrans: [' + ', '.join(fmt(v, n) for v in t) + ']')
    for key in ('extrinsicRot', 'extrinsicRPY'):
        rows = [', '.join(fmt(v, n) for v in row) for row in R]
        pad = ' ' * (len(f'    {key}: ['))
        print(f'    {key}: [{rows[0]},')
        print(f'{pad}{rows[1]},')
        print(f'{pad}{rows[2]}]')


if __name__ == '__main__':
    main()
