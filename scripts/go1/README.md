# Unitree Go1 offline bag conversion

The Go1 data is in ROS 1 bags, and its topics are not what LIO-SAM expects.
These two scripts convert a recording once, offline; every run after that is a
plain `ros2 bag play`. They are ROS 2 ports of FRUC's
`unitree_to_lio_offline_rtk.py` and `go1_lio_preprocessor.py`.

```bash
./unitree_extract.py raw_bags/ -o extracted     # ROS 1 -> ROS 2, HighState -> sensor_msgs/Imu
./go1_preprocess.py  extracted  -o preprocessed # cloud filter + IMU resample to 100 Hz
```

Then run `params_rslidar.yaml` against `preprocessed`, not the original bag.

Needs `rosbags`, `tqdm` and `numpy` (`pip install rosbags tqdm numpy`). No ROS 1
installation is required — ROS 1 bags embed their message definitions, so
[`rosbags`](https://ternaris.gitlab.io/rosbags/) registers
`unitree_legged_msgs/HighState` straight from the file.

---

## Stage 1 — `unitree_extract.py`

Reads one or more ROS 1 bags and writes a single ROS 2 bag. It does only two
things: renames topics, and converts Unitree `HighState` into
`sensor_msgs/Imu` on `/imu_data`. Clouds and GPS pass through untouched.

Several input bags are read in order into one output, since the Unitree
recordings are chunks of a single run. Run it once per bag if you want them
kept separate.

| ROS 1 topic | ROS 2 topic |
|---|---|
| `/rslidar_points` | `/rslidar_points` |
| `/high_state` | `/imu_data` (converted to `sensor_msgs/Imu`) |
| `/camera/imu` | `/camera/imu` |
| `/reach/fix` | `/gps/fix` |
| `/reach/vel` | `/gps/vel` |
| `/reach/time_ref` | `/gps/time_ref` |

`HighState` carries no covariances, so they stay zero.

---

## Stage 2 — `go1_preprocess.py`

Consumes stage 1's bag and produces the topics `params_rslidar.yaml` expects:

| in | out | what happens |
|---|---|---|
| `/rslidar_points` | `/rslidar_points_dense` | non-finite points dropped, range-gated to 0.5–100 m, `ring`/`time` kept, marked `is_dense` |
| `/imu_data` | `/imu_data_resampled` | resampled to 100 Hz, de-biased, EMA-smoothed |
| everything else | unchanged | passed through |

It also renames frames to match the URDF — `rslidar` → `velodyne`, `base` →
`base_link`, `gps` → `navsat_link` — which is what lets `navsat_transform_node`
find the antenna.

The filtering and IMU constants are FRUC's values, at the top of the script.
