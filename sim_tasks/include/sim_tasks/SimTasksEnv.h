#ifndef SIM_TASKS_ENV_H
#define SIM_TASKS_ENV_H

#include <ros/ros.h>
#include "task_manager_lib/TaskDefinition.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseStamped.h"
#include "laser_geometry/laser_geometry.h"
#include "sensor_msgs/Imu.h"
#include "kf_yaw_kf/Compass.h"
#include "nav_msgs/Odometry.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
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
            ros::Subscriber compassSub;
            ros::Subscriber scanSub;
            ros::Publisher velPub;
            ros::ServiceClient muxClient;
            tf::TransformListener listener;

            void buttonCallback(const std_msgs::String::ConstPtr& msg) ;

            void muxCallback(const std_msgs::String::ConstPtr& msg) ;

            void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr msg) ;

            void utmPositionCallback(const nav_msgs::Odometry::ConstPtr& msg) ;

            void compassCallback(const kf_yaw_kf::Compass::ConstPtr& msg) ;

            void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in) ;

            bool manualControl;
            std::string joystick_topic;
            std::string auto_topic;
            std::string position_source;
            pcl::PointCloud<pcl::PointXYZ> pointCloud;
            nav_msgs::Odometry utmPosition;
            kf_yaw_kf::Compass compass;
            geometry_msgs::Pose2D finishLine2D;
            laser_geometry::LaserProjection projector;

        public:
            SimTasksEnv(ros::NodeHandle & nh);
            ~SimTasksEnv() {};

            ros::NodeHandle & getNodeHandle() {return nh;}

            geometry_msgs::Pose2D getPose2D() const ; 

            geometry_msgs::Pose getPose() const ;

            geometry_msgs::PoseStamped getPoseStamped() const  ;

            pcl::PointCloud<pcl::PointXYZ> getPointCloud() const {return pointCloud;}

            void publishVelocity(double linear, double angular) ;

            void setManualControl();
            void setComputerControl();
            bool getManualControl() const {return manualControl;}

            void setFinishLine2D(geometry_msgs::Pose2D pose) {finishLine2D = pose;}
            geometry_msgs::Pose2D getFinishLine2D() const {return finishLine2D;}
    };

};

#endif // SIM_TASKS_ENV_H
