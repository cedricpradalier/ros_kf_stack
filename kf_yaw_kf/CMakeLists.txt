cmake_minimum_required(VERSION 2.4.6)
project(kf_yaw_kf)

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
    rosbuild_genmsg()
    #uncomment if you have defined services
    rosbuild_gensrv()

    # eigen 2 or 3
    find_path(EIGEN_INCLUDE_DIR Eigen/Core
        /usr/local/include/eigen3
        /usr/local/include/eigen2
        /usr/local/include/eigen
        /usr/include/eigen3
        /usr/include/eigen2
        /usr/include/eigen
        /opt/ros/diamondback/stacks/geometry/eigen/include
    )

    # include all libs so far
    include_directories(${EIGEN_INCLUDE_DIR} ${Boost_INCLUDE_DIRS})


    #common commands for building c++ executables and libraries
    #rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
    #target_link_libraries(${PROJECT_NAME} another_library)
    #rosbuild_add_boost_directories()
    #rosbuild_link_boost(${PROJECT_NAME} thread)
    rosbuild_add_executable(kf_yaw_kf src/kf_yaw_kf.cpp)
    target_link_libraries(kf_yaw_kf boost_signals)
else()
    add_definitions(-g -ggdb -O0)
    find_package(catkin REQUIRED
      COMPONENTS
      geometry_msgs
      message_generation
      rospy
      roscpp
      sensor_msgs
      message_filters
    )
    # eigen 2 or 3
    find_path(EIGEN_INCLUDE_DIR Eigen/Core
        /usr/local/include/eigen3
        /usr/local/include/eigen2
        /usr/local/include/eigen
        /usr/include/eigen3
        /usr/include/eigen2
        /usr/include/eigen
        /opt/ros/diamondback/stacks/geometry/eigen/include
    )

    # include all libs so far
    include_directories(${EIGEN_INCLUDE_DIR} ${Boost_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS})

    catkin_python_setup()

    # ROS message generation
    add_message_files(DIRECTORY msg FILES Compass.msg CompassKF.msg PoseKF.msg)
    add_service_files(DIRECTORY srv FILES SetMagOffset.srv)
    generate_messages(DEPENDENCIES geometry_msgs sensor_msgs std_msgs)

    catkin_package(
      CATKIN_DEPENDS
      geometry_msgs
      message_runtime
      sensor_msgs
      # gps_common
      roscpp
      message_filters
    )

    add_executable(kf_yaw_kf src/kf_yaw_kf.cpp)
    add_dependencies(kf_yaw_kf kf_yaw_kf_generate_messages_cpp)
    target_link_libraries(kf_yaw_kf ${catkin_LIBRARIES})

    add_executable(kf_pose_kf src/kf_pose_kf.cpp)
    add_dependencies(kf_pose_kf kf_yaw_kf_generate_messages_cpp)
    target_link_libraries(kf_pose_kf ${catkin_LIBRARIES})

endif()
