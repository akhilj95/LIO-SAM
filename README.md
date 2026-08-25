# LIO-SAM ROS 2 Jazzy

A **ROS 2 Jazzy** port of [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM), carrying the
forestry platform configurations from
[Forestry-Robotics-UC/fruc_lio_sam](https://github.com/Forestry-Robotics-UC/fruc_lio_sam).

Started from upstream's stale `ros2` branch, with the fixes that only ever landed
on `master` brought forward. See `updates.md` for what changed and why.

---

## 1. Dependencies

Tested with **ROS 2 Jazzy on Ubuntu 24.04**.

```bash
sudo apt install ros-jazzy-perception-pcl ros-jazzy-pcl-msgs \
                 ros-jazzy-vision-opencv ros-jazzy-xacro \
                 ros-jazzy-gtsam ros-jazzy-robot-localization \
                 ros-jazzy-rviz2 ros-jazzy-robot-state-publisher
```
Unlike the upstream README, **no GTSAM PPA is needed** — `ros-jazzy-gtsam` is a
released deb and ships `gtsam_unstable` too.

The Go1 bag converters in `scripts/go1/` need Python packages as well:
```bash
pip install rosbags tqdm numpy
```

`scripts/urdf_extrinsics.py` (section 9) additionally needs `numpy` and
`ros-jazzy-urdfdom-py`. Neither is required to build or run the nodes.
---

## 2. Build

Standalone, in any ROS 2 workspace:

```bash
cd ~/ros2_ws
colcon build --packages-select lio_sam --symlink-install
source install/setup.bash
```
---

## 3. Choose a configuration

The parameters file is the only thing to pick. It also selects the URDF and the
navsat topics, so one file sets up the whole stack.

| params file | robot | lidar |
|---|---|---|
| `params.yaml` | upstream sample data | Velodyne VLP-16 |
| `params_curtmini.yaml` | Curt Mini | Ouster OS-1-128 |
| `params_curtmini_flipped.yaml` | Curt Mini, 180° flipped cloud | Ouster OS-1-128 |
| `params_apparatus.yaml` | "apparatus" rig | Ouster OS-128 |
| `params_bunkermini.yaml` | Bunker Mini | Hesai QT128 |
| `params_rslidar.yaml` | Unitree Go1 | RoboSense-32 |

**Two Curt Mini files?** Some bags were decoded with a 180° yaw applied to the
points and some were not, so it is a per-bag choice. Play the bag without TF,
view `/ouster/points` against the URDF, and use whichever file makes the cloud
line up with the robot.

---

## 4. Running

```bash
ros2 launch lio_sam run.launch.py params_file:=params_curtmini.yaml
```

Then play a bag:

```bash
ros2 bag play <bag> --clock --rate 0.3 --exclude /tf /tf_static
```

Excluding the bag's own `/tf` and `/tf_static` matters on recordings that carry
conflicting static transforms — `robot_state_publisher` should be the only
source. Start LIO-SAM before playing the bag.

Launch arguments:

| argument | default | |
|---|---|---|
| `params_file` | `params.yaml` | a name in `config/`, or an absolute path |
| `use_sim_time` | `true` | follows `/clock`; set `false` for live sensors |
| `log_level` | `info` | `lio_sam_mapOptimization` only — `debug` shows the GPS-factor diagnostics |
| `publish_robot_description` | `true` | set `false` if another node already publishes an equivalent robot description |

If the recorded topics differ from the config, either edit `pointCloudTopic` /
`imuTopic` in the params file, or remap at playback:

```bash
ros2 bag play <bag> --clock --remap /imu/data:=/imu/data/corrected
```

---

## 5. Saving the map

```bash
ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap \
  "{resolution: 0.2, destination: /data/maps/}"
```

`resolution` is the voxel size in metres. The map is written as `.pcd`.

---

## 6. Unitree Go1 offline bag conversion

The Go1 data is in ROS 1 bags. `scripts/go1/` ports FRUC's two converters using
the [`rosbags`](https://ternaris.gitlab.io/rosbags/) library, so no ROS 1
installation is needed:

```bash
./unitree_extract.py raw_bags/ -o extracted     # ROS 1 -> ROS 2, HighState -> sensor_msgs/Imu
./go1_preprocess.py  extracted  -o preprocessed # cloud filter + IMU resample to 100 Hz
```

Then run `params_rslidar.yaml` against the result.

---

## 7. Bunker Mini offline bag conversion

Bunker recordings hold raw Hesai UDP rather than point clouds, and the QT128's
timestamps do not match the rest of the bag. Both are handled offline by
`scripts/bunker/convert_bunker.py` — see [`scripts/bunker/README.md`](scripts/bunker/README.md).

Run `params_bunkermini.yaml` against the converted bag, not the original.

---

## 8. Sensor inputs

- LiDAR: `sensor_msgs/PointCloud2`, with `x y z intensity ring time` fields
- IMU: `sensor_msgs/Imu`, 9-axis, 200 Hz or higher

`time` is per-point, relative to the scan start, and FLOAT32 — a driver emitting
an absolute FLOAT64 `timestamp` instead will not be recognised. Lidar and IMU
must also be on the same clock, since LIO-SAM compares their header stamps
directly. Both failures are silent. See upstream's
[prepare lidar data](https://github.com/TixiaoShan/LIO-SAM#prepare-lidar-data)
and [prepare IMU data](https://github.com/TixiaoShan/LIO-SAM#prepare-imu-data),
which still apply unchanged.

All three extrinsics are `T_lb` (lidar ← imu): the rotations map IMU-frame
vectors into lidar axes, and `extrinsicTrans` is the IMU origin expressed in
**lidar** axes.

---

## 9. Deriving the extrinsics from a URDF

`scripts/urdf_extrinsics.py` computes that block for you, so a new platform does
not need the transform chain composed by hand:

```bash
cd config
../scripts/urdf_extrinsics.py ./config/fruc_bunkermini.urdf.xacro \
    --lidar-link hesai_lidar --imu-link imu --snap-deg 1.0
```

It expands the xacro, walks both links up to their common root, and prints
`T_lb = T_root←lidar⁻¹ · T_root←imu` as a paste-ready YAML block, along with the
joint chains it walked and `R_lidar←imu` in degrees so the result can be checked
by eye.

`--lidar-link` is the params file's `lidarFrame`. For IMU link use whatever link 
the IMU driver stamps its messages with.

`--snap-deg DEG` rounds the rotation to the nearest exact axis permutation when
it is within `DEG` of one, and reports how much it discarded. It is **off by default**:
without it you get the raw geometric value. If it declines to snap, the mount is
genuinely off-axis or a joint origin is wrong.

`extrinsicRPY` is emitted equal to `extrinsicRot`, which holds whenever the IMU
reports inertial and attitude data in the same body frame. If yours does not,
override it by hand.

---

## 10. Notes

- Platform URDFs and extrinsics are derived from each robot's own description
  repo, not from the fruc_lio_sam copies, which disagree with them in places.
- **params_rslidar and param_apparatus have not yet been tested.**
- **Velodyne, Livox and Microstrain remain untested here**, as on upstream's
  ROS 2 branch.
- If the map is tilted or unstable, check the TF tree first, then the extrinsics.

---

## Acknowledgement

- LIO-SAM is based on LOAM (J. Zhang and S. Singh, *LOAM: Lidar Odometry and Mapping in Real-time*).
- The ROS 2 migration is TixiaoShan's `ros2` branch and its contributors.
- Platform configurations and the Go1 offline pipeline originate with
  [Forestry-Robotics-UC](https://github.com/Forestry-Robotics-UC/fruc_lio_sam), ISR-UC.
