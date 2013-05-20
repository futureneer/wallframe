cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(ROS_BUILD_TYPE Debug)

rosbuild_init()
include_directories(/usr/include/ni /usr/include/nite)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
rosbuild_add_executable(openni_tracker src/openni_tracker.cpp)
rosbuild_add_executable(openni_tracker src/EnumWrapper.cpp)

target_link_libraries(openni_tracker OpenNI)