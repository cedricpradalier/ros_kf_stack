#include <math.h>
#include "TaskAlignWithShore.h"
#include "sim_tasks_cfg/TaskAlignWithShoreConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace sim_tasks_cfg;
using namespace sim_tasks;

// #define DEBUG_GOTO

TaskIndicator TaskAlignWithShore::iterate()
{

    double vel = 0;
    double rot = 0;

    float mindistance=1000;
    float theta_closest=0;
    float distance_i=0;
    float theta_i=0;
    float angle_error=0;

    const pcl::PointCloud<pcl::PointXYZ> & pointCloud = env->getPointCloud();
    for (unsigned int i=0;i<pointCloud.size();i++) {
        distance_i=hypot(pointCloud[i].y,pointCloud[i].x);
        if ((distance_i < mindistance)&&(distance_i > 0.01)) {
            mindistance=distance_i;
            theta_closest=-atan2(pointCloud[i].y,pointCloud[i].x);
        }
    }

#ifdef DEBUG_GOTO
    ROS_INFO("mindistance %.3f - theta_closest %.3f",mindistance, theta_closest);
#endif
    
    if (mindistance == 1000) {
        ROS_INFO("No laser data");
        env->publishVelocity(0,0);
        return TaskStatus::TASK_RUNNING;
    } else if (mindistance > cfg.dist_threshold) {
        ROS_INFO("No shore detected");
        env->publishVelocity(0,0);
        return TaskStatus::TASK_FAILED;
    } else {
        // This expression guarantee that whatever the error, we always try to
        // align the boat front with the shore
        angle_error = remainder(cfg.angle,2*M_PI)-theta_closest;
        if (fabs(angle_error)<cfg.angle_error) {
            return TaskStatus::TASK_COMPLETED;
        }
        rot = - ((angle_error>0)?+1:-1) * cfg.ang_velocity;
#ifdef DEBUG_GOTO
        ROS_INFO("Command vel %.2f angle_error %.2f rot %.2f\n",vel,angle_error,rot);
#endif
        env->publishVelocity(vel, rot);
        return TaskStatus::TASK_RUNNING;
    }
}

TaskIndicator TaskAlignWithShore::terminate()
{
    env->publishVelocity(0,0);
    return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskFactoryAlignWithShore);
