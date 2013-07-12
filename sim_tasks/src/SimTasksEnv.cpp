
#include "sim_tasks/SimTasksEnv.h"
#include <topic_tools/MuxSelect.h>
#include <boost/algorithm/string.hpp>
#include <laser_geometry/laser_geometry.h>

using namespace sim_tasks;

SimTasksEnv::SimTasksEnv(ros::NodeHandle & n) :
    nh(n), paused(false), manualControl(true), joystick_topic("/teleop/twistCommand"), auto_topic("/mux/autoCommand"), position_source("utm")
{
    // Default value, in case we're running in simulation
    sense.battery = 20.0;
    sense.rc = 0;

    nh.getParam("joystick_topic",joystick_topic);
    nh.getParam("auto_topic",auto_topic);
    nh.getParam("position_source",position_source);

    muxClient = nh.serviceClient<topic_tools::MuxSelect>("/mux/select");

    buttonsSub = nh.subscribe("/buttons",10,&SimTasksEnv::buttonCallback,this);
    muxSub = nh.subscribe("/mux/selected",1,&SimTasksEnv::muxCallback,this);
    pointCloudSub = nh.subscribe("/vrep/hokuyoSensor",1,&SimTasksEnv::pointCloudCallback,this);
    utmPositionSub = nh.subscribe("/gps/utm",1,&SimTasksEnv::utmPositionCallback,this);
    scanSub = nh.subscribe("/lidar/scan",1,&SimTasksEnv::scanCallback,this);
    senseSub = nh.subscribe("/sense",1,&SimTasksEnv::senseCallback,this);
    compassSub = nh.subscribe("/compass/compass",1,&SimTasksEnv::compassCallback,this);
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


geometry_msgs::Pose2D SimTasksEnv::getPose2D() const {
    geometry_msgs::Pose2D pose;
    if (position_source == "tf") {
        tf::StampedTransform transform;
        try{
            listener.lookupTransform("/world","/rosControlledBubbleRob", 
                    ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
        }
        pose.theta = tf::getYaw(transform.getRotation());
        pose.x = transform.getOrigin().x();
        pose.y = transform.getOrigin().y();
    }
    else if (position_source == "utm") {
        const geometry_msgs::Pose & utmPose = utmPosition.pose.pose;
        pose.x = utmPose.position.x;
        pose.y = utmPose.position.y;
        pose.theta = compass.heading;
    } else {
        ROS_ERROR("Parameter position_source undefined");
    }
    return pose;
}

geometry_msgs::Pose SimTasksEnv::getPose() const {
    geometry_msgs::Pose pose;
    if (position_source == "tf") {
        tf::StampedTransform transform;
        try{
            listener.lookupTransform("/world","/rosControlledBubbleRob", 
                    ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
        }
        tf::quaternionTFToMsg(transform.getRotation(),pose.orientation);
        tf::pointTFToMsg(transform.getOrigin(),pose.position);
    } else if (position_source == "utm") {
        pose = utmPosition.pose.pose;
    } else {
        ROS_ERROR("Parameter position_source undefined");
    }
    return pose;
}

geometry_msgs::PoseStamped SimTasksEnv::getPoseStamped() const {
    geometry_msgs::PoseStamped pose;
    if (position_source == "tf") {
        tf::StampedTransform transform;
        try{
            listener.lookupTransform("/world","/rosControlledBubbleRob", 
                    ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
        }
        tf::quaternionTFToMsg(transform.getRotation(),pose.pose.orientation);
        tf::pointTFToMsg(transform.getOrigin(),pose.pose.position);
        pose.header.stamp = transform.stamp_;
    }
    else if (position_source == "utm") {
        pose.pose = utmPosition.pose.pose;
        pose.header = utmPosition.header;
    } else { 
        ROS_ERROR("Parameter position_source undefined");
    }
    return pose;
}

void SimTasksEnv::publishVelocity(double linear, double angular) {
    geometry_msgs::Twist cmd;
    if (paused) {
        cmd.linear.x = 0.;
        cmd.angular.z = 0.;
    } else {
        cmd.linear.x = linear;
        cmd.angular.z = angular;
    }
    velPub.publish(cmd);
}

void SimTasksEnv::buttonCallback(const std_msgs::String::ConstPtr& msg) {
    if (boost::algorithm::to_lower_copy(msg->data) == "pause") {
        paused = !paused;
        if (paused) {
            ROS_INFO("Mission paused");
        } else {
            ROS_INFO("Mission resumed");
        }
    }
}

void SimTasksEnv::muxCallback(const std_msgs::String::ConstPtr& msg) {
    if (msg->data == joystick_topic) {
        manualControl = true;
    } else if (msg->data == auto_topic) {
        manualControl = false;
    } else {
        ROS_ERROR("Received unknown mux selection: '%s'",msg->data.c_str());
    }
}

void SimTasksEnv::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr msg) {
    pointCloud_header = msg->header;
    pcl::fromROSMsg(*msg, pointCloud);
}

void SimTasksEnv::utmPositionCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    utmPosition = *msg;
}

void SimTasksEnv::senseCallback(const kingfisher_msgs::Sense::ConstPtr& msg) {
    sense = *msg;
}

void SimTasksEnv::compassCallback(const kf_yaw_kf::Compass::ConstPtr& msg) {
    compass = *msg;
}

void SimTasksEnv::scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
    pointCloud_header = scan_in->header;
    laser_geometry::LaserProjection projector;
    sensor_msgs::PointCloud2 cloud;
    projector.projectLaser(*scan_in, cloud);
    pcl::fromROSMsg(cloud, pointCloud);
}

