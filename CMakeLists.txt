# ------------------------------------------------------------------------------
#        A Modular Optimization framework for Localization and mApping
#                               (MOLA)
#
# Copyright (C) 2018-2026, Jose Luis Blanco-Claraco, contributors (AUTHORS.md)
# All rights reserved.
# Released under GNU GPL v3. See LICENSE file
# ------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.5)

# Tell CMake we'll use C++ for use in its tests/flags
project(mola_input_rosbag1 LANGUAGES C CXX)

# MOLA CMake scripts: "mola_xxx()"
find_package(mola_common REQUIRED)
# find dependencies:
find_package(mrpt-obs REQUIRED)
find_package(mrpt-maps REQUIRED)
# ROS1 (de)serialization and message definitions are vendored (see ros1/ and
# mrpt_ros_bridge/ subdirectories), so a ROS1 installation is NOT required.
# We only use ROS2 tf2 for the transform tree:
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

find_mola_package(mola_kernel)

# -----------------------
# define lib:
set(LIB_SRCS 
  src/Rosbag1Dataset.cpp
)
set(LIB_PUBLIC_HDRS
  include/mola_input_rosbag1/Rosbag1Dataset.h
)

# This adds non-ROS deps. The rest are handled below with ament_target_dependencies()
mola_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SRCS} ${LIB_PUBLIC_HDRS}
  PRIVATE_LINK_LIBRARIES
    mola::mola_kernel
    mrpt::obs
    mrpt::maps
    mrpt_ros1bridge
    tf2::tf2
    tf2_geometry_msgs::tf2_geometry_msgs
    ${geometry_msgs_TARGETS}
    ${OpenCV_LIBS}
  CMAKE_DEPENDENCIES
    mola_kernel
    mrpt-obs
    #mrpt-ros1bridge  (internal to this package for now)
)

# Add vendored version of ROS1:
# Must be after the library above to avoid polluting its global compiler flags, etc. 
add_subdirectory(ros1 EXCLUDE_FROM_ALL)
target_link_libraries(${PROJECT_NAME} PRIVATE rosbag)

# ---------------------------------------------------
# Add submodule: mrpt_ros1bridge
# ---------------------------------------------------
# It is built here as a private, internal static library, so it does NOT need
# the "mrpt_common" CMake helpers (which are not available as a released
# package): a plain add_library() is enough.

# define lib:
set(LIB_SRCS
  #mrpt_ros_bridge/ros1bridge/src/stereo_image.cpp
  mrpt_ros_bridge/ros1bridge/src/laser_scan.cpp
  #mrpt_ros_bridge/ros1bridge/src/map.cpp
  mrpt_ros_bridge/ros1bridge/src/time.cpp
  mrpt_ros_bridge/ros1bridge/src/gps.cpp
  mrpt_ros_bridge/ros1bridge/src/imu.cpp
  mrpt_ros_bridge/ros1bridge/src/pose.cpp
  #mrpt_ros_bridge/ros1bridge/src/image.cpp
  mrpt_ros_bridge/ros1bridge/src/point_cloud2.cpp
  mrpt_ros_bridge/ros1bridge/src/range.cpp
  mrpt_ros_bridge/ros1bridge/src/point_cloud.cpp
)

set(LIB_PUBLIC_HDRS
  mrpt_ros_bridge/ros1bridge/include/mrpt/ros1bridge/pose.h
  mrpt_ros_bridge/ros1bridge/include/mrpt/ros1bridge/time.h
  mrpt_ros_bridge/ros1bridge/include/mrpt/ros1bridge/range.h
  mrpt_ros_bridge/ros1bridge/include/mrpt/ros1bridge/stereo_image.h
  mrpt_ros_bridge/ros1bridge/include/mrpt/ros1bridge/map.h
  mrpt_ros_bridge/ros1bridge/include/mrpt/ros1bridge/gps.h
  mrpt_ros_bridge/ros1bridge/include/mrpt/ros1bridge/logging.h
  mrpt_ros_bridge/ros1bridge/include/mrpt/ros1bridge/point_cloud.h
  mrpt_ros_bridge/ros1bridge/include/mrpt/ros1bridge/imu.h
  mrpt_ros_bridge/ros1bridge/include/mrpt/ros1bridge/laser_scan.h
  mrpt_ros_bridge/ros1bridge/include/mrpt/ros1bridge/point_cloud2.h
  mrpt_ros_bridge/ros1bridge/include/mrpt/ros1bridge/image.h
)

add_library(mrpt_ros1bridge STATIC ${LIB_SRCS} ${LIB_PUBLIC_HDRS})
# PIC is required because this static lib is linked into the shared ${PROJECT_NAME}:
set_target_properties(mrpt_ros1bridge PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(mrpt_ros1bridge
  PUBLIC
   mrpt::maps
   mrpt::obs
   tf2::tf2
)
target_include_directories(mrpt_ros1bridge
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/mrpt_ros_bridge/ros1bridge/include
)
target_include_directories(${PROJECT_NAME}
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/mrpt_ros_bridge/ros1bridge/include
)

#target_link_libraries(${PROJECT_NAME} PRIVATE Threads::Threads) # for WorkerThreads

# Add the required libraries for linking:
target_link_libraries(mrpt_ros1bridge
  PRIVATE
  rosbag
  #${roscpp_LIBRARIES}
  #${cv_bridge_LIBRARIES}
)
target_include_directories(mrpt_ros1bridge
  SYSTEM
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/ros1/include
)

if (0)
target_include_directories(${PROJECT_NAME}
  SYSTEM
  PUBLIC
    ${tf2_INCLUDE_DIRS}
  PRIVATE
    ${cv_bridge_INCLUDE_DIRS}
)
target_compile_definitions(${PROJECT_NAME} PRIVATE ${ROS_DEFINITIONS})
target_compile_definitions(${PROJECT_NAME} PRIVATE IS_MRPT_ROS1BRIDGE)
target_compile_definitions(${PROJECT_NAME} PRIVATE CV_BRIDGE_VERSION=${cv_bridge_VERSION_HEX})
endif()

# ---------------------------------------------------
# CLI app: "rosbag1-info" (equivalent to ROS1's `rosbag info`)
# ---------------------------------------------------
# A standalone tool that reads a ROS1 .bag and prints its topics, types,
# timing stats, etc. It only needs the vendored rosbag reader, so it works
# in a pure ROS2 (or non-ROS) environment.
# The 'rosbag' target (vendored, see ros1/) publicly exports all the include
# directories we need. mola_add_executable() installs it to bin/, so it lands
# on the PATH once the workspace is sourced.
mola_add_executable(
  TARGET rosbag1-info
  SOURCES apps/rosbag1-info.cpp
  LINK_LIBRARIES rosbag
)

# ---------------------------------------------------



# Convert package versions to hex so they can be used in preprocessor for wider
# versions compatibility of "one-source for all":
#mola_version_to_hex(${cv_bridge_VERSION}  cv_bridge_VERSION_HEX)
#target_compile_definitions(${PROJECT_NAME} PRIVATE CV_BRIDGE_VERSION=${cv_bridge_VERSION_HEX})
#message(STATUS "Found: cv_bridge_VERSION: ${cv_bridge_VERSION} (${cv_bridge_VERSION_HEX})")


