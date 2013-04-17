
#include "sim_tasks/SimTasksEnv.h"
#include <topic_tools/MuxSelect.h>

using namespace sim_tasks;

SimTasksEnv::SimTasksEnv(ros::NodeHandle & n) :
    nh(n), paused(false), manualControl(true), joystick_topic("/teleop/twistCommand"), auto_topic("/mux/autoCommand"), position_source("utm")
{
    nh.getParam("joystick_topic",joystick_topic);
    nh.getParam("auto_topic",auto_topic);
    nh.getParam("position_source",position_source);

    muxClient = nh.serviceClient<topic_tools::MuxSelect>("/mux/select");

    buttonsSub = nh.subscribe("/buttons",10,&SimTasksEnv::buttonCallback,this);
    muxSub = nh.subscribe("/mux/selected",1,&SimTasksEnv::muxCallback,this);
    pointCloudSub = nh.subscribe("/vrep/hokuyoSensor",1,&SimTasksEnv::pointCloudCallback,this);
    utmPositionSub = nh.subscribe("/gps/utm",1,&SimTasksEnv::utmPositionCallback,this);
    scanSub = nh.subscribe("/lidar/scan",1,&SimTasksEnv::scanCallback,this);
    compassSub = nh.subscribe("/imu/data",1,&SimTasksEnv::compassCallback,this);
    velPub = nh.advertise<geometry_msgs::Twist>(auto_topic,1);
}

void SimTasksEnv::setManualControl()
{
    topic_tools::MuxSelect select;
    select.request.topic = joystick_topic;
    if (!muxClient.call(select)) {
        ROS_ERROR("setManualControl: Failed to call service /mux/select");
    }
}

void SimTasksEnv::setComputerControl()
{
    topic_tools::MuxSelect select;
    select.request.topic = auto_topic;
    if (!muxClient.call(select)) {
        ROS_ERROR("setComputerControl: Failed to call service /mux/select");
    }
}


