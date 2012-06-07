cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# create quad_pose_est library

rosbuild_add_library (odom_interface src/odom_interface.cpp)

# create quad_pose_est_nodelet library

rosbuild_add_library (odom_interface_nodelet src/odom_interface_nodelet.cpp)
target_link_libraries (odom_interface_nodelet odom_interface)

# create pid_node executable

rosbuild_add_executable(odom_interface_node src/odom_interface_node.cpp)
target_link_libraries (odom_interface_node odom_interface)
