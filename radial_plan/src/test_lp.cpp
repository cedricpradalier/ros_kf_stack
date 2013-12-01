#include <stdlib.h>
#include <stdio.h>
#include <list>
#include <ros/ros.h>

#include "radial_plan/LocalPlan.h"

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
        pointCloud[i].y = -15 + 2 * sin(pointCloud[i].x*M_PI/4);
    }
    for (unsigned int i=0;i<250;i++) {
        pointCloud[i+250].x = -2.0 + i * 32.0 / 250;
        pointCloud[i+250].y = +10 - 2 * sin(pointCloud[i+250].x*M_PI/4-2.0);
    }
#endif
    FILE * fp = fopen("pc","w");
    for (unsigned int i=0;i<pointCloud.size();i++) {
        fprintf(fp,"%e %e\n",pointCloud[i].x,pointCloud[i].y);
    }
    fclose(fp);


    LocalPlan LP(radial_plan::LocalPlan::RIGHT, 6.0, 2.0, 20.0, 5.0, 0.5, 8, true);
    double t0 = ros::Time::now().toSec();
    LP.updateCellCosts(pointCloud);
    double t1 = ros::Time::now().toSec();
    // LP.saveCellMaps();
    ROS_INFO("updateNodeCosts: %fms",(t1-t0)*1e3);
    
    t0 = ros::Time::now().toSec();
    std::list<cv::Point3f> path;
    path = LP.getOptimalPath(0.5, 0.0, 2.0, 0.5); // tuned from dataset
    t1 = ros::Time::now().toSec();
    ROS_INFO("getOptimalPath: %fms",(t1-t0)*1e3);

    printf("Best path:\n");
    unsigned int i = 0;
    fp = fopen("path","w");
    for (std::list<cv::Point3f>::const_iterator it = path.begin(); it != path.end(); it++, i++) {
        printf("%3d: %6.2f %6.2f %6.2f\n",i,it->x,it->y,it->z);
        fprintf(fp,"%6.2f %6.2f %6.2f\n",it->x,it->y,it->z);
    }
    fclose(fp);

    return 0;
}

