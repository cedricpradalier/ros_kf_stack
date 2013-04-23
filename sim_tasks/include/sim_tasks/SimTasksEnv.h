#ifndef SIM_TASKS_ENV_H
#define SIM_TASKS_ENV_H

#include <ros/ros.h>
#include "task_manager_lib/TaskDefinition.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "kf_yaw_kf/Compass.h"
#include "nav_msgs/Odometry.h"
#include "kingfisher_msgs/Sense.h"
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
            ros::Subscriber senseSub;
            ros::Publisher velPub;
            ros::ServiceClient muxClient;
            tf::TransformListener listener;

            void buttonCallback(const std_msgs::String::ConstPtr& msg) ;

            void senseCallback(const kingfisher_msgs::Sense::ConstPtr& msg) ;

            void muxCallback(const std_msgs::String::ConstPtr& msg) ;

            void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr msg) ;

            void utmPositionCallback(const nav_msgs::Odometry::ConstPtr& msg) ;

            void compassCallback(const kf_yaw_kf::Compass::ConstPtr& msg) ;

            void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in) ;

            bool manualControl;
            std::string joystick_topic;
            std::string auto_topic;
            std::string position_source;
            kingfisher_msgs::Sense sense;
            pcl::PointCloud<pcl::PointXYZ> pointCloud;
            nav_msgs::Odometry utmPosition;
            kf_yaw_kf::Compass compass;
            
            // Specific variable for lake circumnavigation
            geometry_msgs::Pose2D finishLine2D;

        public:
            SimTasksEnv(ros::NodeHandle & nh);
            ~SimTasksEnv() {};

            ros::NodeHandle & getNodeHandle() {return nh;}

            // Returns the current heading in ENU frame, with 0 to the east,
            // pi/2 to the north
            double getHeading() const {
                return compass.heading;
            }

            // Returns the compass angle, defined in NED frame, with 0 to the
            // north, pi/2 to the east
            double getCompass() const {
                return compass.compass;
            }

            double getBattery() const {
                return sense.battery;
            }

            geometry_msgs::Pose2D getPose2D() const ; 

            geometry_msgs::Pose getPose() const ;

            geometry_msgs::PoseStamped getPoseStamped() const  ;

            pcl::PointCloud<pcl::PointXYZ> getPointCloud() const {return pointCloud;}

            void publishVelocity(double linear, double angular) ;

            void setManualControl();
            void setComputerControl();
            bool getManualControl() const {return manualControl || sense.rc;}

            // Specific variable for lake circumnavigation
            void setFinishLine2D(geometry_msgs::Pose2D pose) {finishLine2D = pose;}
            geometry_msgs::Pose2D getFinishLine2D() const {return finishLine2D;}
        public: // To make point cloud work on 32bit system
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };

};

#endif // SIM_TASKS_ENV_H
