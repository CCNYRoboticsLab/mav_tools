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

rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

rosbuild_add_boost_directories()

# create flyer_interface library

rosbuild_add_library (flyer_interface src/serial_communication.cpp
                                      src/flyer_interface.cpp)

rosbuild_link_boost (flyer_interface system)

# create flyer_interface_nodelet library

rosbuild_add_library (flyer_interface_nodelet src/flyer_interface_nodelet.cpp)
target_link_libraries (flyer_interface_nodelet flyer_interface)

# create flyer_interface_node executable

rosbuild_add_executable(flyer_interface_node src/flyer_interface_node.cpp)
target_link_libraries (flyer_interface_node flyer_interface)
