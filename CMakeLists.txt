# ------------------------------------------------------------------------------
#        A Modular Optimization framework for Localization and mApping
#                               (MOLA)
#
# Copyright (C) 2018-2025, Jose Luis Blanco-Claraco, contributors (AUTHORS.md)
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
#find_package(mrpt-ros1bridge REQUIRED)
#find_package(tf2_ros REQUIRED)
#find_package(nav_msgs REQUIRED)
#find_package(sensor_msgs REQUIRED)
#find_package(rosbag2_cpp REQUIRED)
#find_package(tf2_msgs REQUIRED)
#find_package(cv_bridge REQUIRED)
#find_package(tf2_geometry_msgs REQUIRED)

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
    #mrpt::ros1bridge
    #tf2::tf2
    #rosbag2_cpp::rosbag2_cpp
    #tf2_geometry_msgs::tf2_geometry_msgs
    #tf2_msgs::tf2_msgs__rosidl_typesupport_cpp
    #sensor_msgs::sensor_msgs_library
    #cv_bridge::cv_bridge
  CMAKE_DEPENDENCIES
    mola_kernel
    mrpt-obs
    #mrpt-ros1bridge  (internal to this package for now)
)

# Add vendored version of ROS1:
# Must be after the library above to avoid polluting its global compiler flags, etc. 
add_subdirectory(ros1 EXCLUDE_FROM_ALL)
target_link_libraries(${PROJECT_NAME} PRIVATE rosbag)

# Add submodule: mrpt_ros1bridge
add_subdirectory(mrpt_ros_bridge/ros1bridge EXCLUDE_FROM_ALL)
#target_link_libraries(${PROJECT_NAME} PRIVATE mrpt::mrpt_ros1bridge)

# Convert package versions to hex so they can be used in preprocessor for wider
# versions compatibility of "one-source for all":
#mola_version_to_hex(${cv_bridge_VERSION}  cv_bridge_VERSION_HEX)
#target_compile_definitions(${PROJECT_NAME} PRIVATE CV_BRIDGE_VERSION=${cv_bridge_VERSION_HEX})
#message(STATUS "Found: cv_bridge_VERSION: ${cv_bridge_VERSION} (${cv_bridge_VERSION_HEX})")


