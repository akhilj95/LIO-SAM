This branch starts from TixiaoShan's `ros2` branch at
[`08af3f3`](https://github.com/TixiaoShan/LIO-SAM/commit/08af3f3) and does two
things: it brings the ROS 2 code back in line with ROS 1 `master`, which the
`ros2` branch had drifted from, and it adds the FRUC robots — which in ROS 1
lived in launch-file XML — as self-contained ROS 2 parameter files.

## Menu

- [**Lineage**](#lineage)
- [**At a glance**](#at-a-glance)
- [**Fixes to the upstream ros2 branch**](#fixes-to-the-upstream-ros2-branch)
- [**Brought forward from ROS 1 master**](#brought-forward-from-ros-1-master)
- [**What came from FRUC**](#what-came-from-fruc)

## Lineage

The important asymmetry: **upstream's `ros2` branch forked from `master` before
several fixes landed, and FRUC forked after.** FRUC's source is essentially
current `master` — it has `pubSLAMInfo`, `saveMapService`, `matP` as a member,
and `extQRPY = Quaterniond(extRPY).inverse()` — while the `ros2` branch has
none of those. So porting FRUC to ROS 2 was not only a matter of translating
launch files; the ROS 2 code itself had to be caught up first, otherwise a FRUC
config would silently mean something different here than it does on their robot.

## At a glance

| | ROS 1 `master` | upstream `ros2` | FRUC (ROS 1) | **this branch** |
|---|---|---|---|---|
| ROS version | Noetic | Foxy–Humble | Noetic | **Jazzy** |
| Launch | `run.launch` + 4 `module_*.launch` | one `run.launch.py` | `run.launch` + 4 `module_*.launch` | **one `run.launch.py`** |
| GPS / navsat module | yes | **missing** | yes | **yes** |
| `robot_state_publisher` | yes | yes | yes | yes |
| Platform configs | 1 (`params.yaml`) | 1 | 3 (+ 5 URDFs) | **5 (+ 5 URDFs)** |
| `slam_info` topic (`ab97ea3`) | yes | **no** | yes | **yes** |
| `extQRPY` inverse (`df05f8a`) | yes | **no** | yes | **yes** |
| `matP` persists across LM iters | yes | **no** | yes | **yes** |
| `save_map` == `savePCD` output | yes | **no** | yes | **yes** |
| Loop-closure corrected cloud topic | own topic | **collides** | own topic | **own topic** |
| Offline Go1 bag converters | — | — | ROS 1 only | **ROS 2 (`rosbags`)** |

## Fixes to the upstream ros2 branch

These are ROS 2-specific defects, not present in ROS 1 in any form.

**`odom -> base_link` was broadcast with an empty `frame_id`** ([`7a5e050`](../../commit/7a5e050)).
The header was populated only inside the `lidarFrame == baselinkFrame` branch,
so tf2 rejected every transform with `TF_NO_FRAME_ID` on any robot where the two
frames differ — which is all of them here.

**Both `imuPreintegration` nodes were renamed to the same thing** ([`9ea6825`](../../commit/9ea6825)).
That process constructs two `rclcpp::Node`s (`TransformFusion` and
`IMUPreintegration`). `Node(name=...)` in a ROS 2 launch file becomes a
process-wide `--ros-args -r __node:=` remap, so both came up identically named
and collided on the rosout logger and their parameter services. In ROS 1 the
same two objects existed, but `ParamServer` held a `ros::NodeHandle` rather than
being a node, so upstream's identical `name=` was unambiguous.

**Three RViz displays were silently dead** ([`f26aff3`](../../commit/f26aff3)).
`utility.hpp` publishes several topics `BEST_EFFORT`, but `rviz2.rviz` carried
RViz's default `Reliable` for every display. A `RELIABLE` subscription never
connects to a `BEST_EFFORT` publisher and ROS 2 reports nothing when that
happens.

**`lidar_link` had two parents** ([`4c3110a`](../../commit/4c3110a)).
`mapOptimization` broadcasts `odom -> lidar_link`, and the ROS 2 URDF *also*
declared it as a fixed child of `base_link`. The ROS 1 URDF omits it for exactly
this reason.

**`find_package(Eigen)`** ([`7a5e050`](../../commit/7a5e050)) — `Eigen3Config`
sets `EIGEN3_INCLUDE_DIRS` while `ament_target_dependencies` looks for
`Eigen_INCLUDE_DIRS`. Now `Eigen3::Eigen` is linked directly.

## Brought forward from ROS 1 master

**`extrinsicRPY` handling** ([`dc5b0d8`](../../commit/dc5b0d8)) — upstream
[`df05f8a`](https://github.com/TixiaoShan/LIO-SAM/commit/df05f8a). `extrinsicRPY`
is defined as `T_lb`, so the quaternion applied to the IMU orientation is its
*inverse*. `master` and the `ros2` branch each hold one half of this and both
evaluate correctly on their own — the trap is mixing them, which leaves the IMU
orientation transform inverted, feeding `imuRPYWeight` and heading
initialization. This branch adopts `master`'s convention so that FRUC configs,
which are derived from `master`, mean here what they mean there.

**`matP` reset every LM iteration** ([`f0c352b`](../../commit/f0c352b)).
The ROS 2 migration changed the `matP` member from `cv::Mat` to
`Eigen::Matrix<float,6,6>` but left both use sites in `LMOptimization` operating
on `cv::Mat`, and added a function-local `cv::Mat matP` to keep it compiling —
shadowing the member. `LMOptimization` computes `matP` only on `iterCount == 0`
and relies on it persisting. With a local, degenerate scans got **one** usable
iteration instead of thirty.

**`slam_info` publishing** ([`d13cda5`](../../commit/d13cda5)) — upstream
`ab97ea3`. ROS 1 serializes the clouds by calling `publishCloud()` with an empty
`ros::Publisher`; that does not translate, so `toCloudMsg()` was added
(templated, since key frame poses are `PointTypePose`).

**`savePCD` on shutdown** ([`8092cd6`](../../commit/8092cd6)) — the ROS 2 port
turned the service into a lambda but left the shutdown thread on pre-`#317`
inline code, which had drifted in four ways (ASCII not binary, different cloud
names, downsampled with *mapping* leaf sizes rather than full density, and
`mkdir` without `-p`).

**Loop-closure corrected cloud** ([`4668434`](../../commit/4668434)) —
`pubIcpKeyFrames` was created on the same topic as `pubHistoryKeyFrames`, so
subscribers got the historical submap and the ICP-aligned scan interleaved.

## What came from FRUC

FRUC carries per-robot geometry in `launch/include/config/*.urdf.xacro`, selected
by XML in `run.launch`. ROS 2 launch has no clean equivalent, and the ROS 1
arrangement let a config be paired with a URDF it was never meant for. Here each
robot is **one parameters file that names its own URDF** ([`c26a3a8`](../../commit/c26a3a8)):

```yaml
urdfFile:        "fruc_curtmini.urdf.xacro"   # robot model, relative to config/
navsatImuTopic:  "/imu/data"                  # navsat_transform_node's 'imu' input
gpsFixTopic:     "/fix"                       # its 'gps/fix' input
```

No node declares these three, so they are ignored as parameters and read only by
`run.launch.py`. Pairing a config with the wrong robot is no longer possible.

**The navsat remap trap.** `navsat_transform_node` hardcodes its subscriptions
in C++ (`"imu"`, `"gps/fix"`, `"odometry/filtered"`), so they can only be changed
by remapping — unlike `ekf_node`, which takes its input topics as parameters
(`imu0`, `odom0`). ROS 1 subscribed to `imu/data`; **ROS 2 subscribes to bare
`imu`.** A literally-ported ROS 1 remap therefore matches nothing and fails
silently: no IMU → `transform_good_` is never set → `/odometry/gps` is never
published → `addGPSFactor()` never fires, with no error anywhere.

**Not ported, deliberately:**

| FRUC script | why not |
|---|---|
| `ouster_filter.py` | Existed because ROS 1 LIO-SAM calls `ros::shutdown()` on any cloud with `is_dense == false`. The ROS 2 branch already calls `pcl::removeNaNFromPointCloud()` in `cachePointCloud()` (upstream `9d039fd`). Python over a 128×2048 cloud at 20 Hz would cost real time for nothing. |
| `gps_tf_broadcaster.py` | Needed live because FRUC never renamed their bag frames. The converters here rename to match the URDF, so `robot_state_publisher` supplies the antenna offset. |
| `static_transforms.py` | Superseded by the URDF + `run.launch.py`. |