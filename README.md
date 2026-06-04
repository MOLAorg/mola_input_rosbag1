# mola_input_rosbag1

| Distro | Build dev | Release |
| --- | --- | --- |
| ROS 2 Humble (u22.04) | [![Build Status](https://build.ros2.org/job/Hdev__mola_input_rosbag1__ubuntu_jammy_amd64/badge/icon)](https://build.ros2.org/job/Hdev__mola_input_rosbag1__ubuntu_jammy_amd64/) | [![Version](https://img.shields.io/ros/v/humble/mola_input_rosbag1)](https://index.ros.org/?search_packages=true&pkgs=mola_input_rosbag1) |
| ROS 2 Jazzy (u24.04) | [![Build Status](https://build.ros2.org/job/Jdev__mola_input_rosbag1__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Jdev__mola_input_rosbag1__ubuntu_noble_amd64/) | [![Version](https://img.shields.io/ros/v/jazzy/mola_input_rosbag1)](https://index.ros.org/?search_packages=true&pkgs=mola_input_rosbag1) |
| ROS 2 Kilted (u24.04) | [![Build Status](https://build.ros2.org/job/Kdev__mola_input_rosbag1__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Kdev__mola_input_rosbag1__ubuntu_noble_amd64/) | [![Version](https://img.shields.io/ros/v/kilted/mola_input_rosbag1)](https://index.ros.org/?search_packages=true&pkgs=mola_input_rosbag1) |
| ROS 2 Rolling (u24.04) | [![Build Status](https://build.ros2.org/job/Rdev__mola_input_rosbag1__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Rdev__mola_input_rosbag1__ubuntu_noble_amd64/) | [![Version](https://img.shields.io/ros/v/rolling/mola_input_rosbag1)](https://index.ros.org/?search_packages=true&pkgs=mola_input_rosbag1) |

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
| `sensor_msgs/Image`       | `CObservationImage` (mono8, rgb8, bgr8) |
| `sensor_msgs/PointCloud2` | `CObservationPointCloud` (XYZ / XYZI / XYZIRT) or `CObservationRotatingScan` |
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
  `fixed_sensor_pose` (see the example in `ros1bag_just_view.yaml`).

## Parameters

| Param | Required | Default | Description |
|-------|----------|---------|-------------|
| `rosbag_filename`    | yes | -          | Path to the input `.bag` file. |
| `base_link_frame_id` | no  | `base_link`| Robot body frame for tf pose lookup. |
| `time_warp_scale`    | no  | `1.0`      | Playback speed multiplier. |
| `read_ahead_length`  | no  | `15`       | Number of messages pre-read ahead. |
| `start_paused`       | no  | `false`    | Start playback paused. |
| `sensors`            | no  | auto       | Explicit list of topics, types, and pose overrides. If omitted, all topics with a known mapping are exposed automatically using the topic name as `sensorLabel`. |

See `mola-cli-launchs/ros1bag_just_view.yaml` for a fully documented `sensors`
example.

## Demos

Two ready-to-use `mola-cli` launch files are provided under
`mola-cli-launchs/`.

### 1. Visualize raw streams

```bash
ROSBAG1_FILE=/path/to/dataset.bag \
  mola-cli src/mola_input_rosbag1/mola-cli-launchs/ros1bag_just_view.yaml
```

### 2. ROS 1 -> ROS 2 bridge

```bash
ROSBAG1_FILE=/path/to/dataset.bag \
  mola-cli src/mola_input_rosbag1/mola-cli-launchs/ros1bag_to_ros2.yaml
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
  mola-cli src/mola_input_rosbag1/mola-cli-launchs/ros1bag_to_ros2.yaml
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
