#include <stdlib.h>
#include <stdio.h>
#include <list>
#include <ros/ros.h>
#include <tf/tf.h>

#include "radial_plan/RadialPlan.h"

#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <laser_geometry/laser_geometry.h>


using namespace radial_plan;

ros::Publisher pathPub;
ros::Subscriber pcSub;
RadialPlan RP(10, 15, 11, 1.0, M_PI);

void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
    // ROS_INFO("Received point cloud");
    pcl::PointCloud<pcl::PointXYZ> pointCloud;
    laser_geometry::LaserProjection projector;
    sensor_msgs::PointCloud2 cloud;
    projector.projectLaser(*scan_in, cloud);
    pcl::fromROSMsg(cloud, pointCloud);
    RP.updateNodeCosts(pointCloud, 6.0, 2.0);
    std::list<cv::Point2f> lpath;
    lpath = RP.getOptimalPath(1.0, 1.0, 1.0, 10.0);
    // Here we could add some path optimisation, in particular using the 
    // search datastructure on the point cloud: 
    // nss = RP.getNearestNeighbourSearch()
    //
    // TODO
    //
    // Finally create a ROS path message
    nav_msgs::Path path;
    path.header = scan_in->header;
    path.poses.resize(lpath.size());
    std::list<cv::Point2f>::const_iterator it = lpath.begin();
    unsigned int ipose = 0;
    while (it != lpath.end()) {
        // time stamp is not updated because we're not creating a
        // trajectory at this stage
        path.poses[ipose].header = path.header;
        path.poses[ipose].pose.position.x = it->x;
        path.poses[ipose].pose.position.y = it->y;
        if (ipose > 0) {
            const geometry_msgs::Point & prev = path.poses[ipose-1].pose.position;
            tf::Quaternion Q = tf::createQuaternionFromRPY(0,0,atan2(it->y-prev.y,it->x-prev.x));
            tf::quaternionTFToMsg(Q,path.poses[ipose].pose.orientation);
        } else {
            tf::Quaternion Q = tf::createQuaternionFromRPY(0,0,0);
            tf::quaternionTFToMsg(Q,path.poses[ipose].pose.orientation);
        }
        ipose++;
        it ++;
    }
    pathPub.publish(path);
    // ROS_INFO("Request completed");

}



int main(int argc, char *argv[]) {
    ros::init(argc,argv,"live_rp");
    ros::NodeHandle nh("~");
    pathPub = nh.advertise<nav_msgs::Path>("path",1);
    pcSub = nh.subscribe("/lidar/scan",1,scanCallback);
    ros::spin();
    
    return 0;
}

