cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE MinSizeRel)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# create laser_height_estimation library

rosbuild_add_library (laser_height_estimation src/laser_height_estimation.cpp)

# create laser_height_estimation_nodelet library

rosbuild_add_library (laser_height_estimation_nodelet src/laser_height_estimation_nodelet.cpp)
target_link_libraries (laser_height_estimation_nodelet laser_height_estimation)

# create laser_height_estimation executable

rosbuild_add_executable(laser_height_estimation_node src/laser_height_estimation_node.cpp)
target_link_libraries (laser_height_estimation_node laser_height_estimation)
