cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
# set(ROS_BUILD_TYPE RelWithDebInfo)
# set(ROS_BUILD_TYPE Debug)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#add dynamic reconfigure api
# Uncomment once we have the first task
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()


#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

# FIND_PACKAGE(Ceres REQUIRED)
#INCLUDE_DIRECTORIES(${CERES_INCLUDES})

#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME} src/RadialPlan.cpp src/LocalPlan.cpp)
rosbuild_add_boost_directories()
rosbuild_add_executable(test_rp src/test_rp.cpp)
target_link_libraries(test_rp ${PROJECT_NAME} ${CERES_LIBRARIES})

rosbuild_add_executable(test_lp src/test_lp.cpp)
target_link_libraries(test_lp ${PROJECT_NAME} ${CERES_LIBRARIES})

rosbuild_add_executable(live_rp src/live_rp.cpp)
target_link_libraries(live_rp ${PROJECT_NAME} ${CERES_LIBRARIES})

rosbuild_add_executable(live_lp src/live_lp.cpp)
rosbuild_link_boost(live_lp thread)
target_link_libraries(live_lp ${PROJECT_NAME} ${CERES_LIBRARIES})
