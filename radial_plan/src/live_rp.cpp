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
RadialPlan RP(10, 15, 11, false, 1.0, M_PI);
double dt_update=0.0, dt_path=0.0;
unsigned long num_iter = 0;

void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
    // ROS_INFO("Received point cloud");
    pcl::PointCloud<pcl::PointXYZ> pointCloud;
    laser_geometry::LaserProjection projector;
    sensor_msgs::PointCloud2 cloud;
    projector.projectLaser(*scan_in, cloud);
    pcl::fromROSMsg(cloud, pointCloud);
    double t0 = ros::Time::now().toSec();
    RP.updateNodeCosts(pointCloud, RadialPlan::LEFT, 10.0, 2.0);
    double t1 = ros::Time::now().toSec();
    std::list<cv::Point3f> lpath;
    lpath = RP.getOptimalPath(1.0, 1.0, 1.0, 10.0);
    double t2 = ros::Time::now().toSec();
    dt_update += t1 - t0;
    dt_path += t2 - t1;
    num_iter += 1;
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
    std::list<cv::Point3f>::const_iterator it = lpath.begin();
    size_t ipose = 0;
    while (it != lpath.end()) {
        // time stamp is not updated because we're not creating a
        // trajectory at this stage
        path.poses[ipose].header = path.header;
        path.poses[ipose].pose.position.x = it->x;
        path.poses[ipose].pose.position.y = it->y;
        tf::Quaternion Q = tf::createQuaternionFromRPY(0,0,it->z);
        tf::quaternionTFToMsg(Q,path.poses[ipose].pose.orientation);
        it ++;
        ipose ++;
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

    printf("Average computation time: %ld iter, update %.2f ms, path %.2f\n",num_iter, 1e3*dt_update/num_iter, 1e3*dt_path/num_iter);
    
    return 0;
}

