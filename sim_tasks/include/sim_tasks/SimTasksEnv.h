#ifndef SIM_TASKS_ENV_H
#define SIM_TASKS_ENV_H

#include <ros/ros.h>
#include "task_manager_lib/TaskDefinition.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "boost/algorithm/string.hpp"
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

namespace sim_tasks {
    class SimTasksEnv: public task_manager_lib::TaskEnvironment
    {
        protected:
            ros::NodeHandle nh;
            bool paused;
            ros::Subscriber buttonsSub;
            ros::Subscriber muxSub;
            ros::Subscriber pointCloudSub;
			ros::Subscriber utmPositionSub;
            ros::Publisher velPub;
            ros::ServiceClient muxClient;
            tf::TransformListener listener;

            void buttonCallback(const std_msgs::String::ConstPtr& msg) {
                if (boost::algorithm::to_lower_copy(msg->data) == "pause") {
                    paused = !paused;
                    if (paused) {
                        ROS_INFO("Mission paused");
                    } else {
                        ROS_INFO("Mission resumed");
                    }
                }
            }

            void muxCallback(const std_msgs::String::ConstPtr& msg) {
                if (msg->data == joystick_topic) {
                    manualControl = true;
                } else if (msg->data == auto_topic) {
                    manualControl = false;
                } else {
                    ROS_ERROR("Received unknown mux selection: '%s'",msg->data.c_str());
                }
            }

            void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr msg) {
                pcl::fromROSMsg(*msg, pointCloud);
            }

            void utmPositionCallback(const nav_msgs::Odometry::ConstPtr& msg) {
                utmPosition = *msg;
            }

            bool manualControl;
            std::string joystick_topic;
            std::string auto_topic;
            std::string position_source;
            pcl::PointCloud<pcl::PointXYZ> pointCloud;
            nav_msgs::Odometry utmPosition;
            geometry_msgs::Pose2D finishLine2D;

        public:
            SimTasksEnv(ros::NodeHandle & nh);
            ~SimTasksEnv() {};

            ros::NodeHandle & getNodeHandle() {return nh;}

            geometry_msgs::Pose2D getPose2D() const {
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
                    pose.theta = tf::getYaw(utmPose.orientation);
                } else {
                    ROS_ERROR("Parameter position_source undefined");
                }
                return pose;
            }

            geometry_msgs::Pose getPose() const {
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

            geometry_msgs::PoseStamped getPoseStamped() const {
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

            pcl::PointCloud<pcl::PointXYZ> getPointCloud() const {return pointCloud;}

            void publishVelocity(double linear, double angular) {
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

            void setManualControl();
            void setComputerControl();
            bool getManualControl() const {return manualControl;}

            void setFinishLine2D(geometry_msgs::Pose2D pose) {finishLine2D = pose;}
            geometry_msgs::Pose2D getFinishLine2D() const {return finishLine2D;}
    };

};

#endif // SIM_TASKS_ENV_H
