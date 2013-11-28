#include <stdlib.h>
#include <stdio.h>
#include <list>
#include <ros/ros.h>

#include "radial_plan/RadialPlan.h"

#include <pcl_ros/point_cloud.h>

using namespace radial_plan;

int main(int argc, char *argv[]) {
    ros::Time::init();

    pcl::PointCloud<pcl::PointXYZ> pointCloud;
    pointCloud.resize(500);
#if 0
    for (unsigned int i=0;i<250;i++) {
        pointCloud[i].x = -2.0 + i * 32.0 / 250;
        pointCloud[i].y = -12.0 + pointCloud[i].x * 3.0;
    }
    for (unsigned int i=0;i<250;i++) {
        pointCloud[i+250].x = -2.0 + i * 32.0 / 250;
        pointCloud[i+250].y = 12.0 - pointCloud[i+250].x * 0.40;
    }
#else
    for (unsigned int i=0;i<250;i++) {
        pointCloud[i].x = -2.0 + i * 32.0 / 250;
        pointCloud[i].y = -5 + 2 * sin(pointCloud[i].x*M_PI/4);
    }
    for (unsigned int i=0;i<250;i++) {
        pointCloud[i+250].x = -2.0 + i * 32.0 / 250;
        pointCloud[i+250].y = +5 - 2 * sin(pointCloud[i+250].x*M_PI/4-2.0);
    }
#endif

    RadialPlan RP(10, 15, 11, false, 1.0, M_PI);
    double t0 = ros::Time::now().toSec();
    RP.updateNodeCosts(pointCloud, RadialPlan::RIGHT, 6.0, 4.0);
    double t1 = ros::Time::now().toSec();
    ROS_INFO("updateNodeCosts: %fms",(t1-t0)*1e3);
    
    t0 = ros::Time::now().toSec();
    std::list<cv::Point2f> path;
    path = RP.getOptimalPath(0.1, 1.0, 0.1, 100.0);
    t1 = ros::Time::now().toSec();
    ROS_INFO("getOptimalPath: %fms",(t1-t0)*1e3);

    printf("Best path:\n");
    unsigned int i = 0;
    for (std::list<cv::Point2f>::const_iterator it = path.begin(); it != path.end(); it++, i++) {
        printf("%3d: %6.2f %6.2f\n",i,it->x,it->y);
    }

    return 0;
}

