^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mola_input_rosbag1
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


0.3.0 (2026-07-31)
------------------
* fix clang-format
* Add use_bag_record_time option for drivers with unsynced header stamps
  Some drivers (observed on TIERS's Ouster and Livox streams) publish
  header.stamp from an internal/relative clock never synced to the
  recording PC's wall clock, which silently breaks ground-truth time
  lookup for every scan. When set on a sensor, the observation's
  timestamp is taken from the bag's own message storage time instead.
* Add support for livox_ros_driver2/CustomMsg too
* Update mrpt_ros_bridge submodule: drop deprecated point-map overloads
  Removes the CPointsMapXYZI/CPointsMapXYZIRT fromROS/toROS overloads,
  which were the source of -Wdeprecated-declarations warnings when
  building Rosbag1Dataset.cpp; CGenericPointsMap already covers the
  same conversions.
* launch: add per-app imgui_app_name for MolaVizImGui
  Sets a unique imgui_app_name in each launch file so Dear ImGui's
  layout persistence stores a separate UI configuration per app.
* Merge pull request `#1 <https://github.com/MOLAorg/mola_input_rosbag1/issues/1>`_ from MOLAorg/feature/multi-bag-gt-and-livox-custommsg
  Multi-bag input, ground-truth trajectory API, and Livox CustomMsg support
* fix: validate nav_msgs/Odometry GT quaternion; migrate toPointCloud2 off deprecated point map classes
  - Apply the same non-finite-quaternion guard already used for
  geometry_msgs/PoseStamped to the nav_msgs/Odometry ground-truth branch.
  - Update mrpt_ros_bridge submodule to a commit adding CGenericPointsMap
  fromROS/toROS overloads for sensor_msgs/PointCloud2, and switch
  toPointCloud2() to use them instead of the deprecated
  CPointsMapXYZI/CPointsMapXYZIRT classes.
* refactor: use CGenericPointsMap instead of deprecated CPointsMapXYZIRT in toLivoxCustomMsg
* feat: multi-bag input, ground-truth trajectory API, and Livox CustomMsg support
  - 'rosbag_filename' now accepts a YAML sequence of paths (merged and replayed
  jointly in time order), enabling datasets that ship sensors and ground
  truth in separate .bag files (e.g. BotanicGarden).
  - New 'ground_truth_topic' param pre-scans a geometry_msgs/PoseStamped or
  nav_msgs/Odometry topic into a full trajectory_t, exposed via the
  mola::OfflineDatasetSource ground-truth API (hasGroundTruthTrajectory() /
  getGroundTruthTrajectory()), in addition to normal per-step publishing.
  Malformed poses (e.g. non-finite quaternions) are skipped with a warning
  instead of aborting.
  - Added a livox_ros_driver/CustomMsg -> CObservationPointCloud converter
  (offset_time -> per-point time, reflectivity -> intensity, line -> ring),
  so Livox AVIA-style LiDARs publishing this message type are supported.
  - New mola-cli-launchs/rosbag1_botanicgarden.yaml example wiring all of the
  above for the BotanicGarden dataset's LIO + ground-truth bags.
* feat: support PoseStamped messages
* docs: add ros2 bridge example
* Contributors: Jose Luis Blanco-Claraco

0.2.0 (2026-06-10)
------------------
* Update mrpt_ros_bridge submodule
* fix: build of ros1 headers in gcc15
* fix: enforce c++17 so build doesn't fail with GCC-15+
* Add loader for NTU viral dataset
* Contributors: Jose Luis Blanco-Claraco

0.1.0 (2026-06-10)
------------------
* fix: don't crash for missing topic names
* feat: support compressed images and more image encoding formats
* feat: add rosbag1-info CLI app
* CI: make Kilted a required build (deps available; only Rolling still allowed to fail)
* CI: don't let unreleased-distro builds (kilted, rolling) block the matrix
* CI: build on all active ROS 2 distros (add Kilted and Rolling)
* Use distro-independent 'bzip2' rosdep key (not available as libbz2-dev on humble)
* Declare system deps for the vendored ROS1 rosbag reader (BZip2, lz4, Boost)
* Fix CI build: resolve rosdep keys and vendored bridge build
* Finish ROS1 bag input module: working reader, demos, docs and CI
* import ROS1 bag library
* Continue backporting to ROS1
* First skeleton based on ROS2 bag source
* Contributors: Jose Luis Blanco-Claraco

0.0.1 (2026-05-01)
------------------
