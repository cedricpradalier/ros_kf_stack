
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <laser_geometry/laser_geometry.h>


class CollisionAvoidance {
    protected:
        ros::Subscriber scanSub;
        ros::Subscriber pcSub;
        ros::Subscriber disableSub;
        ros::Subscriber velSub;
        ros::Publisher velPub;

        ros::NodeHandle nh;
        double lateral_tolerance, distance_max_reverse, distance_max_forward, distance_zero, forward_speed, reverse_speed;
        bool allow_reverse;
        double xmin;
        ros::Time disable_stamp, pc_stamp;

        pcl::PointCloud<pcl::PointXYZ> lastpc;

        void velocity_filter(const geometry_msgs::TwistConstPtr msg) {
            geometry_msgs::Twist filtered = findClosestAcceptableVelocity(*msg);
            velPub.publish(filtered);
        }

        void scan_callback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
        {
            pc_stamp = scan_in->header.stamp;
            laser_geometry::LaserProjection projector;
            sensor_msgs::PointCloud2 cloud;
            projector.projectLaser(*scan_in, cloud);
            pcl::fromROSMsg(cloud, lastpc);
            updateClosestPoint();
        }

        void pc_callback(const sensor_msgs::PointCloud2ConstPtr msg) {
            pc_stamp = msg->header.stamp;
            pcl::fromROSMsg(*msg, lastpc);
            updateClosestPoint();
        }

        void updateClosestPoint() {
            xmin = 1e6;
            unsigned int n = lastpc.size();
            for (unsigned int i=0;i<n;i++) {
                double x = lastpc[i].x;
                double y = lastpc[i].y;

                if (hypot(x,y) < 1e-2) {
                    // bogus point, the laser did not return
                    continue;
                }

                if (fabs(y) > lateral_tolerance) {
                    // too far on the side
                    continue;
                }
                if (x < 0) {
                    // not going towards it
                    continue;
                }

                xmin = std::min(xmin,x);
            }
        }

        void disable_callback(const std_msgs::BoolConstPtr msg) {
            if (msg->data) {
                ROS_WARN("Laser safety: temporarily disabled");
                disable_stamp = ros::Time::now();
            } else {
                disable_stamp = ros::Time();
            }
        }

        geometry_msgs::Twist findClosestAcceptableVelocity(const geometry_msgs::Twist & desired) {
            geometry_msgs::Twist res = desired;
            ros::Time now = ros::Time::now();
            if ((now-disable_stamp).toSec() < 1.0) {
                // Accept to be temporarily disabled
                return res;
            }
            if (desired.linear.x <= 0.0) {
                // Ignore reverse motions
                return res;
            }
            if ((now-pc_stamp).toSec() > 1.0) {
                // No data, setting velocity to zero
                ROS_WARN("Laser safety: no data received, setting velocity to zero");
                res.linear.x = 0.0;
                return res;
            }
            if (xmin <= distance_zero) {
                if (allow_reverse) {
                    xmin = std::max(xmin,distance_max_reverse);
                    res.angular.z = 0.0;
                    res.linear.x = reverse_speed * (distance_zero - xmin) / (distance_zero - distance_max_reverse);
                } else {
                    res.linear.x = 0.0;
                }
            } else if (xmin < distance_max_forward) {
                res.linear.x = std::min(desired.linear.x, forward_speed * (xmin - distance_zero) / (distance_max_forward - distance_zero));
            }
            // ROS_INFO("Speed limiter: desired %.2f controlled %.2f",desired.linear.x,res.linear.x);

            return res;
        }

    public:
        CollisionAvoidance() : nh("~") {

            nh.param("lateral_tolerance",lateral_tolerance,1.0);
            nh.param("allow_reverse",allow_reverse,true);
            nh.param("distance_max_reverse",distance_max_reverse,1.0);
            nh.param("distance_zero",distance_zero,1.5);
            nh.param("distance_max_forward",distance_max_forward,3.0);
            nh.param("forward_speed",forward_speed,1.0);
            nh.param("reverse_speed",reverse_speed,-1.0);
            

            scanSub = nh.subscribe("scan",1,&CollisionAvoidance::scan_callback,this);
            pcSub = nh.subscribe("pointcloud",1,&CollisionAvoidance::pc_callback,this);
            velSub = nh.subscribe("input",1,&CollisionAvoidance::velocity_filter,this);
            disableSub = nh.subscribe("disable",1,&CollisionAvoidance::disable_callback,this);
            velPub = nh.advertise<geometry_msgs::Twist>("output",1);

        }

};

int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"collision_avoidance");

    CollisionAvoidance ca;

    ros::spin();
}


