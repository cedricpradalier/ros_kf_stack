cmake_minimum_required(VERSION 2.4.6)
project(axis_exposure_control)

if(USE_ROSBUILD)
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
    rosbuild_add_executable(axis_exposure_control src/axis_exposure_control.cpp)
    #target_link_libraries(example ${PROJECT_NAME})
else()

  find_package(catkin REQUIRED 
    COMPONENTS roscpp sensor_msgs image_transport cv_bridge tf)

    find_package(OpenCV REQUIRED)

  catkin_package(
    CATKIN_DEPENDS roscpp sensor_msgs image_transport cv_bridge tf
    )

  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    )

    add_executable(axis_exposure_control src/axis_exposure_control.cpp)
    target_link_libraries(axis_exposure_control ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

    install(TARGETS axis_exposure_control 
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

endif()
