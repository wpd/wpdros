cmake_minimum_required(VERSION 2.4.6)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Debug)
rosbuild_init()
rosbuild_genmsg()

rosbuild_add_boost_directories()

# move_base
rosbuild_add_executable(bin/wpd_move_base src/move_base.cpp)
rosbuild_link_boost(bin/wpd_move_base system)
