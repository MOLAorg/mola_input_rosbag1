# mola_input_rosbag1

| Distro | Build dev | Builds | Stable release |
| --- | --- | --- | --- |
| ROS 2 Humble (u22.04) | [![Build Status](https://build.ros2.org/job/Hdev__mola_input_rosbag1__ubuntu_jammy_amd64/badge/icon)](https://build.ros2.org/job/Hdev__mola_input_rosbag1__ubuntu_jammy_amd64/) | amd64 [![Build Status](https://build.ros2.org/job/Hbin_uJ64__mola_input_rosbag1__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__mola_input_rosbag1__ubuntu_jammy_amd64__binary/) <br> arm64 [![Build Status](https://build.ros2.org/job/Hbin_ujv8_uJv8__mola_input_rosbag1__ubuntu_jammy_arm64__binary/badge/icon)](https://build.ros2.org/job/Hbin_ujv8_uJv8__mola_input_rosbag1__ubuntu_jammy_arm64__binary/)  | [![Version](https://img.shields.io/ros/v/humble/mola_input_rosbag1)](https://index.ros.org/?search_packages=true&pkgs=mola_input_rosbag1) |
| ROS 2 Jazzy (u24.04) | [![Build Status](https://build.ros2.org/job/Jdev__mola_input_rosbag1__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Jdev__mola_input_rosbag1__ubuntu_noble_amd64/) | | [![Version](https://img.shields.io/ros/v/jazzy/mola_input_rosbag1)](https://index.ros.org/?search_packages=true&pkgs=mola_input_rosbag1) |
| ROS 2 Kilted (u24.04) | [![Build Status](https://build.ros2.org/job/Kdev__mola_input_rosbag1__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Kdev__mola_input_rosbag1__ubuntu_noble_amd64/) |  |[![Version](https://img.shields.io/ros/v/kilted/mola_input_rosbag1)](https://index.ros.org/?search_packages=true&pkgs=mola_input_rosbag1) |
| ROS 2 Lyrical (u26.04) | [![Build Status](https://build.ros2.org/job/Ldev__mola_input_rosbag1__ubuntu_resolute_amd64/badge/icon)](https://build.ros2.org/job/Ldev__mola_input_rosbag1__ubuntu_resolute_amd64/) | amd64 [![Build Status](https://build.ros2.org/job/Lbin_uR64__mola_input_rosbag1__ubuntu_resolute_amd64__binary/badge/icon)](https://build.ros2.org/job/Lbin_uR64__mola_input_rosbag1__ubuntu_resolute_amd64__binary/) <br> arm64 [![Build Status](https://build.ros2.org/job/Lbin_unv8_uRv8__mola_input_rosbag1__ubuntu_resolute_arm64__binary/badge/icon)](https://build.ros2.org/job/Lbin_unv8_uRv8__mola_input_rosbag1__ubuntu_resolute_arm64__binary/) | [![Version](https://img.shields.io/ros/v/lyrical/mola_input_rosbag1)](https://index.ros.org/?search_packages=true&pkgs=mola_input_rosbag1) |
| ROS 2 Rolling (u26.04) | [![Build Status](https://build.ros2.org/job/Rdev__mola_input_rosbag1__ubuntu_resolute_amd64/badge/icon)](https://build.ros2.org/job/Rdev__mola_input_rosbag1__ubuntu_resolute_amd64/) | amd64 [![Build Status](https://build.ros2.org/job/Rbin_uR64__mola_input_rosbag1__ubuntu_resolute_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uR64__mola_input_rosbag1__ubuntu_resolute_amd64__binary/) <br> arm64 [![Build Status](https://build.ros2.org/job/Rbin_unv8_uRv8__mola_input_rosbag1__ubuntu_resolute_arm64__binary/badge/icon)](https://build.ros2.org/job/Rbin_unv8_uRv8__mola_input_rosbag1__ubuntu_resolute_arm64__binary/) | [![Version](https://img.shields.io/ros/v/rolling/mola_input_rosbag1)](https://index.ros.org/?search_packages=true&pkgs=mola_input_rosbag1) |

A MOLA `RawDataSource` module that reads **ROS 1 `.bag` files** and exposes their
contents as MOLA observations, **without requiring a ROS 1 installation**.

The ROS 1 bag (de)serialization, message definitions, and `rosbag_storage`
reader are *vendored* inside this package (see the `ros1/` and
`mrpt_ros_bridge/` subdirectories), so the module builds and runs in a pure
ROS 2 (or even non-ROS) colcon workspace.

Typical uses:

- Feed a legacy ROS 1 dataset into **MOLA LiDAR Odometry** or any other MOLA
  consumer.
- Turn MOLA into a **ROS 1 -> ROS 2 bridge**: read a `.bag` and re-publish its
  streams as ROS 2 topics + `/tf`, in real time (see demo below).
- Inspect raw sensor streams in the MOLA visualizer.

## Supported message types

| ROS 1 message type        | MOLA / MRPT observation        |
|---------------------------|--------------------------------|
| `sensor_msgs/Imu`         | `CObservationIMU`              |
| `sensor_msgs/Image`       | `CObservationImage` (mono8, mono16, rgb8, bgr8, rgba8, bgra8, bayer_rggb8, bayer_bggr8, bayer_gbrg8, bayer_grbg8) |
| `sensor_msgs/CompressedImage` | `CObservationImage` (JPEG, PNG, and any format supported by OpenCV `imdecode`) |
| `sensor_msgs/PointCloud2` | `CObservationPointCloud` (XYZ / XYZI / XYZIRT) or `CObservationRotatingScan` |
| `livox_ros_driver/CustomMsg`, `livox_ros_driver2/CustomMsg` | `CObservationPointCloud` (XYZIRT: intensity=reflectivity, ring=line, time=offset_time) |
| `sensor_msgs/LaserScan`   | `CObservation2DRangeScan`      |
| `sensor_msgs/NavSatFix`   | `CObservationGPS`              |
| `nav_msgs/Odometry`       | `CObservationOdometry`         |
| `tf2_msgs/TFMessage` (`/tf`, `/tf_static`) | transform tree (sensor pose lookup) |

Topics with no known mapping are ignored (a one-time warning is logged).

## Sensor poses and `/tf`

The pose of each sensor in the robot body frame is looked up from the bag's
`/tf` and `/tf_static` tree, as the transform
`base_link_frame_id` -> `<message frame_id>`.

- Set `base_link_frame_id` to the name of your robot body frame. Note that many
  ROS 1 bags use *namespaced* frames (e.g. `r1/base_link` instead of
  `base_link`).
- If the relevant transform is not yet available when a message is read (e.g.
  the first few messages before any `/tf`), that single observation is dropped
  and a throttled warning lists the currently known tf frames, which is handy
  for discovering the correct `base_link_frame_id`.
- If your bag has no `/tf`, override the pose per sensor with
  `fixed_sensor_pose` (see the example in `rosbag1_lidar.yaml`).

## Parameters

| Param | Required | Default | Description |
|-------|----------|---------|-------------|
| `rosbag_filename`    | yes | -          | Path to the input `.bag` file, **or a YAML sequence of paths** to merge and replay jointly in time order (e.g. a sensors bag plus a separate ground-truth-only bag). |
| `base_link_frame_id` | no  | `base_link`| Robot body frame for tf pose lookup. |
| `time_warp_scale`    | no  | `1.0`      | Playback speed multiplier. |
| `read_ahead_length`  | no  | `15`       | Number of messages pre-read ahead. |
| `start_paused`       | no  | `false`    | Start playback paused. |
| `sensors`            | no  | auto       | Explicit list of topics, types, and pose overrides. If omitted, all topics with a known mapping are exposed automatically using the topic name as `sensorLabel`. |
| `ground_truth_topic` | no  | -          | Topic (`geometry_msgs/PoseStamped` or `nav_msgs/Odometry`) pre-scanned at start-up into a full trajectory, exposed via the `mola::OfflineDatasetSource` ground-truth API (`hasGroundTruthTrajectory()` / `getGroundTruthTrajectory()`), e.g. for `evo`-style evaluation. This is independent from (and can be combined with) listing the same topic under `sensors` for normal per-timestep publishing as a `CObservationRobotPose`/`CObservationOdometry`. |

See `mola-cli-launchs/rosbag1_lidar.yaml` for a fully documented `sensors`
example, and `mola-cli-launchs/rosbag1_botanicgarden.yaml` for an example
combining multiple input bags with `ground_truth_topic`.

## Demos

Two ready-to-use `mola-cli` launch files are provided under
`mola-cli-launchs/`.

### 1. Visualize raw LiDAR streams

```bash
ROSBAG1_FILE=/path/to/dataset.bag \
  mola-cli src/mola_input_rosbag1/mola-cli-launchs/rosbag1_lidar.yaml
```

### 2. ROS 1 -> ROS 2 bridge

Example for the NTU Viral dataset:

```bash
ROSBAG1_FILE=/mnt/storage/ntu-viral/eee_01/eee_01.bag \
mola-cli \
  src/mola_input_rosbag1/mola-cli-launchs/rosbag1_ntu_viral.yaml \
  src/mola_input_rosbag1/mola-cli-launchs/publish_to_ros2.yam
```

Then, in another terminal with ROS 2 sourced:

```bash
ros2 topic list
ros2 topic echo /your_topic
rviz2
```

If your bag uses namespaced frames, point the body frame at the right one, e.g.:

```bash
MOLA_BASE_LINK_FRAME=r1/base_link \
ROSBAG1_FILE=/path/to/dataset.bag \
  mola-cli src/mola_input_rosbag1/mola-cli-launchs/rosbag1_to_ros2.yaml
```

## CLI tools

### `rosbag1-info`

An equivalent to ROS 1's `rosbag info`, but built on the vendored ROS 1 bag
reader so it runs in a pure ROS 2 (or non-ROS) environment. Once this package
is built and the workspace sourced, the tool is on the `PATH`:

```bash
rosbag1-info /path/to/dataset.bag
```

It prints the bag path and version, recording duration, start/end wall-clock
times, file size, total message count, compression, the message types (with
their md5sums), and, per topic, the message count, average frequency and type:

```
path:        /path/to/dataset.bag
version:     2.0
duration:    208.8s (3min 28s)
start:       Oct 18 2022 06:12:24.58 (1666066344.58)
end:         Oct 18 2022 06:15:53.36 (1666066553.36)
size:        1.9 GB
messages:    150843
compression: none
types:       nav_msgs/Odometry [cd5e73d190d741a2f92e81eda573aca7]
             sensor_msgs/Imu [6a62c6daae103f4ff57a132d6f95cec2]
             sensor_msgs/PointCloud2 [1158d486dd51d683ce2f1be655c3c181]
topics:      /imu/data          73279 msgs @   399.6 Hz : sensor_msgs/Imu
             /odom              37195 msgs @   200.0 Hz : nav_msgs/Odometry
             /velodyne_points    2071 msgs @     9.9 Hz : sensor_msgs/PointCloud2
```

## Building

```bash
cd ~/ros2_ws
colcon build --packages-select mola_input_rosbag1
```

The vendored ROS 1 reader needs Boost, BZip2 and lz4 development packages
(already present in a standard ROS 2 desktop install).

## Credits

- The vendored ROS 1 `rosbag_storage` / `roscpp_core` sources under `ros1/` are
  taken from [Traj-LO](https://github.com/kevin2431/Traj-LO)
  (originally Willow Garage, BSD license).
- ROS <-> MRPT message conversions come from the
  [mrpt_ros_bridge](https://github.com/MRPT/mrpt_ros_bridge) `ros1` branch,
  vendored as a git submodule.
- Bag iteration logic is adapted from MRPT's `rosbag2rawlog`.
