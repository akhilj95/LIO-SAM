# LIO-SAM — ROS 2 Jazzy, FRUC forestry platforms

A ROS 2 Jazzy port of [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM) carrying the
platform configurations from [Forestry-Robotics-UC/fruc_lio_sam](https://github.com/Forestry-Robotics-UC/fruc_lio_sam).



## Menu
- [**Dependencies and build**](#dependencies-and-build)
- [**Running**](#running)
- [**Frames and extrinsics**](#frames-and-extrinsics)
- [**Platforms**](#platforms)
- [**Offline bag conversion**](#offline-bag-conversion)
- [**Known gaps**](#known-gaps)
- [**Measured throughput**](#measured-throughput)
- [**Upstream documentation**](#upstream-documentation)
- [**Paper**](#paper)


## Dependencies and build

Jazzy on Ubuntu 24.04. Unlike the upstream README, **no GTSAM PPA is needed** —
`ros-jazzy-gtsam` is a released deb, and the `borglab/gtsam-release-4.1` PPA the
upstream instructions use has no Noble build.

```bash
sudo apt install ros-jazzy-perception-pcl ros-jazzy-pcl-msgs \
                 ros-jazzy-vision-opencv ros-jazzy-xacro \
                 ros-jazzy-gtsam ros-jazzy-robot-localization
```

## Running

The parameters file is the only thing to choose:

```bash
ros2 launch lio_sam run.launch.py params_file:=params_curtmini.yaml
```

| argument | default | |
|---|---|---|
| `params_file` | `params.yaml` | a name in `config/`, or an absolute path |
| `use_sim_time` | `true` | follows `/clock`; set `false` for live sensors |
| `log_level` | `info` | scoped to `lio_sam_mapOptimization` only — `debug` surfaces the GPS-factor gate diagnostics without rclcpp internals |

Replay bag file:

```bash
ros2 bag play <bag> --clock --rate 0.3 --exclude /tf /tf_static
```

Excluding the bag's own `/tf` and `/tf_static` matters on recordings that carry
conflicting static transforms; `robot_state_publisher` should be the only source.

## Frames and extrinsics

Two things are easy to conflate, and the distinction is the whole reason
`params_curtmini.yaml` looks the way it does:

- **`lidarFrame` is a label.** It stamps outgoing messages and drives
  TransformFusion's `lidar -> baselink` lookup. It never transforms the incoming
  cloud.
- **`extrinsicRot` / `extrinsicTrans` are pinned to the data**, not to the label.

## Platforms

| params file | robot | lidar | URDF |
|---|---|---|---|
| `params.yaml` | upstream sample data | Velodyne VLP-16 | `robot.urdf.xacro` |
| `params_curtmini.yaml` | Curt Mini | Ouster OS-1-128 | `fruc_curtmini.urdf.xacro` |
| `params_ouster.yaml` | "apparatus" rig | Ouster OS-128 | `fruc_ouster.urdf.xacro` |
| `params_hesai.yaml` | Bunker Mini | Hesai-128 | `fruc_hesai.urdf.xacro` |
| `params_rslidar.yaml` | Unitree Go1 | RoboSense-32 | `fruc_rslidar.urdf.xacro` |

## Offline bag conversion

`scripts/go1/` ports FRUC's two offline converters ([`9f83b1e`](../../commit/9f83b1e)).
They use the [`rosbags`](https://ternaris.gitlab.io/rosbags/) library, which
reads ROS 1 bags and writes ROS 2 bags with no ROS 1 installation — so the
pipeline stays in this workspace instead of needing a Noetic container.

```bash
./unitree_extract.py raw_bags/ -o extracted     # ROS 1 -> ROS 2, HighState -> sensor_msgs/Imu
./go1_preprocess.py  extracted  -o preprocessed # cloud filter + IMU resample to 100 Hz
```

`bagio.py` holds the shared message rebuilding, structured-array cloud access and
mcap writing.

## Known gaps

Documented rather than papered over:

- **`params_hesai.yaml` extrinsics are inconsistent with FRUC's own URDF.** Its
  `extrinsicTrans` matches `oak_imu_frame -> hesai_lidar` — the OAK camera's IMU,
  not the Xsens, and in the reversed direction — while the rotation matches
  nothing in `sensor.urdf.xacro`. `fruc_hesai.urdf.xacro` still has placeholder
  zeros. Untested; do not trust this config without re-deriving it.
- **`bunkermini.urdf.xacro` is broken as shipped upstream** — it includes
  `sensors.urdf.xacro`, but the file is named `sensor.urdf.xacro`.
- **Velodyne, Livox and Microstrain remain untested here**, as on upstream's
  ROS 2 branch.

## Upstream documentation

The following sections of the [upstream README](https://github.com/TixiaoShan/LIO-SAM/blob/master/README.md)
apply unchanged and are not duplicated here: system architecture, **prepare
lidar data**, **prepare IMU data**, sample datasets, and the service/topic
reference. The IMU sections in particular are still required reading — the
extrinsic conventions there are what everything above depends on.


## Acknowledgement

- LIO-SAM is based on LOAM (J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time).
- The ROS 2 migration is TixiaoShan's `ros2` branch and its contributors.
- Platform configurations and the Go1 offline pipeline originate with
  [Forestry-Robotics-UC](https://github.com/Forestry-Robotics-UC/fruc_lio_sam).