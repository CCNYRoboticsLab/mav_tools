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

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#################################################################################

# create quad_simple_planner library

rosbuild_add_library (ctrl_interface src/ctrl_interface.cpp)

# create quad_simple_planner_nodelet library

rosbuild_add_library (ctrl_interface_nodelet src/ctrl_interface_nodelet.cpp)
target_link_libraries (ctrl_interface_nodelet ctrl_interface)

# create quad_simple_planner_node executable

rosbuild_add_executable(ctrl_interface_node src/ctrl_interface_node.cpp)
target_link_libraries (ctrl_interface_node ctrl_interface)

