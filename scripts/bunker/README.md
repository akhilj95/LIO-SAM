# Bunker Mini offline bag conversion

`convert_bunker.py` turns a raw Bunker Mini recording into a bag LIO-SAM can
play directly. Run it once per recording; every run after that is a plain
`ros2 bag play`.

```bash
./convert_bunker.py ~/data/<input-bag-name> -o ~/data/<output-bag-name>
```

---

## Why it is needed

A Bunker recording cannot be fed to LIO-SAM as-is, for three independent
reasons. Each one is silent on its own, which is what makes them worth
documenting.

**1. There are no point clouds.** The recorder captured raw UDP on
`/hesai/lidar_packets` (`hesai_ros_driver/msg/UdpFrame`), not `PointCloud2`.
Decoding is only reachable through the vendored HesaiLidar driver — so this 
script drives the real driver rather than reimplementing the parser.

**2. The lidar clock is years off.** The QT128 had no PTP/GPS lock, so its
packets carry a free-running internal clock while `/imu/data` and `/fix` are on
the recorder's host clock. LIO-SAM compares those stamps directly.

**3. The time field is the wrong name, type and origin.** The driver emits
`timestamp` as FLOAT64 *absolute* seconds. LIO-SAM's velodyne path registers
`(float, time, time)` and its field scan accepts only the names `time` or `t`,
so `timestamp` is not recognised. Worse than the warning it logs: PCL never
populates `time`, so it stays `0`, `timeScanEnd == timeScanCur`, and every scan
looks instantaneous. The IMU-coverage check then passes trivially and the run
looks healthy while doing no motion compensation at all.

---

## What it produces

| topic | |
|---|---|
| `/lidar_points/corrected` | decoded clouds, re-stamped onto the bag clock, with a relative FLOAT32 `time` field |
| `/imu/data` | copied through byte-exact |
| `/fix` | copied through byte-exact |

The IMU and GPS never traverse the ROS graph — they are copied as raw
serialized bytes straight from the source bag, interleaved in timestamp order,
so they cannot be dropped or re-timed. Only the cloud is modified.

Zero-return points are dropped by default: roughly half of each frame is exact
`(0,0,0)` padding that `lidarMinRange` discards anyway, so keeping it only makes
the bag bigger and every later run slower. Use `--keep-empty` to retain them.

The output topic is deliberately **not** the driver's native `/lidar_points`, so
a converted bag cannot be confused with, or replayed alongside, raw driver
output.

---

## Prerequisites

The HesaiLidar ROS 2 driver must be built in the same workspace. **It is not
vendored in this repository**. The `Dockerfile` has a commented section if
you want to use that.

```bash
git clone --recurse-submodules https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0.git
```

Its `config/config.yaml` must be set for packet replay. The script checks all of
this before starting rather than producing an empty bag:

```yaml
source_type: 3                       # packet rosbag
use_timestamp_type: 0                # 1 zeroes every timestamp on this source
rosbag_type:
  correction_file_path: ".../correction/angle_correction/QT128C2X_Angle Correction File.csv"
  firetimes_path:       ".../correction/firetime_correction/QT128C2X_Firetime Correction File.csv"
ros:
  ros_recv_packet_topic: /hesai/lidar_packets
```

Both correction files ship with the driver's SDK submodule. They are the generic
per-model files, not this unit's factory calibration — the bag carries no
`/lidar_corrections` topic, so there is nothing unit-specific to recover.

> **The driver reads that file from its *source* tree at runtime, not from
> `install/`.** `PROJECT_PATH` is baked in at compile time and
> `RUN_IN_ROS_WORKSPACE` is ROS 1 only, so `config_path` resolves to
> `<src>/HesaiLidar_ROS_2.0/config/config.yaml`. Edits take effect with no
> rebuild — and editing the copy under `share/` does nothing at all.

---

## Options

| option | |
|---|---|
| `-o, --output` | output bag directory (required) |
| `--offset` | clock offset in seconds. Derived automatically when omitted |
| `--calib-frames` | frames used to derive the offset (default 15) |
| `--rate` | playback rate (default 1.0) |
| `--duration` | convert only the first N seconds — useful for a quick check |
| `--keep-empty` | keep zero-return `(0,0,0)` points |
| `--hesai-config` | path to the driver's `config.yaml` |
| `--drain-timeout` | seconds without a cloud after playback before closing |
| `--force` | overwrite an existing output bag |

The offset is derived by pairing the first `--calib-frames` `UdpFrame` stamps
against the decoded cloud stamps and taking the **median**, which absorbs the
occasional dropped frame — a dropped one would otherwise show up as a single
pair displaced by a whole scan period. The derived value and its spread are
logged; a spread above 50 ms warns, since that indicates the pairing broke.

---

## Gotchas

**Only one driver may run at a time.** A leftover instance publishes on the same
topic, which silently doubles the apparent frame rate. To check by hand:

```bash
pgrep -af lib/hesai_ros_driver/
```

**Do not raise `--rate` much above 1.0.** The driver's subscription queue is 10
deep; outrunning it drops packets, which corrupts frames rather than merely
losing them. A full 390 s recording takes about 7 minutes.

---

## Verifying the result

```bash
ros2 bag info <path-to-bag>
```

Expect the cloud count to match the packet count in the source bag, and the
start time to sit on the *host* clock rather than the lidar's. Then check one
cloud:

- fields are `x y z intensity ring(u16) time(f32)`, `point_step 24`
- `time` spans roughly `0.0 → 0.1` for a 10 Hz sensor
- the header stamp is within milliseconds of the source bag's `UdpFrame` stamp
  for the same revolution

If LIO-SAM logs `Point cloud timestamp not available, deskew function disabled`,
the `time` conversion did not take effect and the run will drift.
