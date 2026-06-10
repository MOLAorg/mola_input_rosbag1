^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mola_input_rosbag1
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


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
