cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

ROSBUILD_ADD_EXECUTABLE( task_server src/task_server.cpp src/SimTasksEnv.cpp) 
TARGET_LINK_LIBRARIES(task_server dl boost_system)

# Now compile the tasks independently

ROSBUILD_ADD_LIBRARY( TaskSetHeading SHARED tasks/TaskSetHeading.cpp)
TARGET_LINK_LIBRARIES(TaskSetHeading dl)

ROSBUILD_ADD_LIBRARY( TaskCalibrateCompass SHARED tasks/TaskCalibrateCompass.cpp)
TARGET_LINK_LIBRARIES(TaskCalibrateCompass dl)

ROSBUILD_ADD_LIBRARY( TaskGoTo SHARED tasks/TaskGoTo.cpp)
TARGET_LINK_LIBRARIES(TaskGoTo dl)

ROSBUILD_ADD_LIBRARY( TaskGoToAngle SHARED tasks/TaskGoToAngle.cpp)
TARGET_LINK_LIBRARIES(TaskGoToAngle dl)

ROSBUILD_ADD_LIBRARY( TaskFollowShore SHARED tasks/TaskFollowShore.cpp)
TARGET_LINK_LIBRARIES(TaskFollowShore dl)

ROSBUILD_ADD_LIBRARY( TaskAlignWithShore SHARED tasks/TaskAlignWithShore.cpp)
TARGET_LINK_LIBRARIES(TaskAlignWithShore dl)

ROSBUILD_ADD_LIBRARY( TaskRecordFinishLine SHARED tasks/TaskRecordFinishLine.cpp)
TARGET_LINK_LIBRARIES(TaskRecordFinishLine dl)

rOSBUILD_ADD_LIBRARY( TaskReachOtherShore SHARED tasks/TaskReachOtherShore.cpp)
TARGET_LINK_LIBRARIES(TaskReachOtherShore dl)

ROSBUILD_ADD_LIBRARY( TaskWaitForAuto SHARED tasks/TaskWaitForAuto.cpp)
TARGET_LINK_LIBRARIES(TaskWaitForAuto dl)

ROSBUILD_ADD_LIBRARY( TaskSetManual SHARED tasks/TaskSetManual.cpp)
TARGET_LINK_LIBRARIES(TaskSetManual dl)

ROSBUILD_ADD_LIBRARY( TaskSetPTZ SHARED tasks/TaskSetPTZ.cpp)
TARGET_LINK_LIBRARIES(TaskSetPTZ dl)

ROSBUILD_ADD_LIBRARY( TaskFollowShorePID SHARED tasks/TaskFollowShorePID.cpp)
TARGET_LINK_LIBRARIES(TaskFollowShorePID dl)

ROSBUILD_ADD_LIBRARY( TaskFollowShoreRP SHARED tasks/TaskFollowShoreRP.cpp)
TARGET_LINK_LIBRARIES(TaskFollowShoreRP dl)

ROSBUILD_ADD_LIBRARY( TaskSetOrigin SHARED tasks/TaskSetOrigin.cpp)
TARGET_LINK_LIBRARIES(TaskSetOrigin dl)

ROSBUILD_ADD_LIBRARY( TaskWaitForROI SHARED tasks/TaskWaitForROI.cpp)
TARGET_LINK_LIBRARIES(TaskWaitForROI dl)

ROSBUILD_ADD_LIBRARY( TaskWaitForDistance SHARED tasks/TaskWaitForDistance.cpp)
TARGET_LINK_LIBRARIES(TaskWaitForDistance dl)

ROSBUILD_ADD_LIBRARY( TaskConstant SHARED tasks/TaskConstant.cpp)
TARGET_LINK_LIBRARIES(TaskConstant dl)
