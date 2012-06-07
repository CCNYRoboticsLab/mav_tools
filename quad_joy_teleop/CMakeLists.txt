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

#rosbuild_gensrv()

# create quad_joy_teleop library

rosbuild_add_library (quad_joy_teleop src/quad_joy_teleop.cpp)

# create quad_joy_teleop_nodelet library

rosbuild_add_library (quad_joy_teleop_nodelet src/quad_joy_teleop_nodelet.cpp)
target_link_libraries (quad_joy_teleop_nodelet quad_joy_teleop)

# create quad_joy_teleop_node executable

rosbuild_add_executable(quad_joy_teleop_node src/quad_joy_teleop_node.cpp)
target_link_libraries (quad_joy_teleop_node quad_joy_teleop)
