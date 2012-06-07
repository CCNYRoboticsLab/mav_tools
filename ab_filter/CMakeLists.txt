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

#################################################################################

# create ab_filter_height library
rosbuild_add_library (ab_filter_height src/ab_filter_height.cpp)

# create ab_filter_height_nodelet library
rosbuild_add_library (ab_filter_height_nodelet src/ab_filter_height_nodelet.cpp)
target_link_libraries (ab_filter_height_nodelet ab_filter_height)

# create ab_filter_height_node executable
rosbuild_add_executable(ab_filter_height_node src/ab_filter_height_node.cpp)
target_link_libraries (ab_filter_height_node ab_filter_height)

#################################################################################

# create ab_filter_pose library
rosbuild_add_library (ab_filter_pose src/ab_filter_pose.cpp)

# create ab_filter_pose_nodelet library
rosbuild_add_library (ab_filter_pose_nodelet src/ab_filter_pose_nodelet.cpp)
target_link_libraries (ab_filter_pose_nodelet ab_filter_pose)

# create ab_filter_height_node executable
rosbuild_add_executable(ab_filter_pose_node src/ab_filter_pose_node.cpp)
target_link_libraries (ab_filter_pose_node ab_filter_pose)


