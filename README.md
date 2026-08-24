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

## 7. Sensor inputs

- LiDAR: `sensor_msgs/PointCloud2`, with `x y z intensity ring time` fields
- IMU: `sensor_msgs/Imu`, 9-axis, 200 Hz or higher

All three extrinsics are `T_lb` (lidar ← imu): the rotations map IMU-frame
vectors into lidar axes, and `extrinsicTrans` is the IMU origin expressed in
**lidar** axes. See also the **prepare IMU data** section of the
[upstream README](https://github.com/TixiaoShan/LIO-SAM/blob/master/README.md),
which still applies unchanged.

---

## 8. Notes

- Platform URDFs and extrinsics are derived from each robot's own description
  repo, not from the fruc_lio_sam copies, which disagree with them in places.
- **Bunker Mini is untested against a bag.** Its frames and extrinsics are
  verified, but `Horizon_SCAN`, `pointCloudTopic` and the presence of a `ring`
  field are inherited from upstream and unconfirmed. See the header of
  `params_bunkermini.yaml`.
- **Velodyne, Livox and Microstrain remain untested here**, as on upstream's
  ROS 2 branch.
- If the map is tilted or unstable, check the TF tree first, then the extrinsics.

---

## Acknowledgement

- LIO-SAM is based on LOAM (J. Zhang and S. Singh, *LOAM: Lidar Odometry and Mapping in Real-time*).
- The ROS 2 migration is TixiaoShan's `ros2` branch and its contributors.
- Platform configurations and the Go1 offline pipeline originate with
  [Forestry-Robotics-UC](https://github.com/Forestry-Robotics-UC/fruc_lio_sam), ISR-UC.
