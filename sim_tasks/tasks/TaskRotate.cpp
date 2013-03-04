#include <math.h>
#include "TaskRotate.h"
#include "sim_tasks/TaskRotateConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace sim_tasks;

#define DEBUG_GOTO

TaskRotate::TaskRotate(boost::shared_ptr<TaskEnvironment> tenv) 
    : TaskDefinitionWithConfig<TaskRotateConfig,TaskRotate>("Rotate","Rotate till a certain angle",true,-1.)
{
    env = boost::dynamic_pointer_cast<SimTasksEnv,TaskEnvironment>(tenv);
}

TaskIndicator TaskRotate::iterate()
{
    const pcl::PointCloud<pcl::PointXYZ> & pointCloud = env->getPointCloud();

    double vel = 0;
    double rot = 0;

    float mindistance=1000;
    float theta_closest=0;
    float distance_i=0;
    float theta_i=0;

    for (unsigned int i=0;i<pointCloud.size();i++) {
        theta_i=atan2(pointCloud[i].y,pointCloud[i].x);
        if (remainder(cfg.angle-theta_i,2*M_PI)<cfg.angle_range) {
            distance_i=hypot(pointCloud[i].y,pointCloud[i].x);
            if ((distance_i < mindistance)&&(distance_i > 0.01)) {
	            mindistance=distance_i;
                theta_closest=theta_i;
            }
        }
    }

#ifdef DEBUG_GOTO
    ROS_INFO("mindistance %.3f - theta_closest %.3f",mindistance, theta_closest);
#endif
    
    float angle_error=0;

    angle_error = remainder(cfg.angle-theta_closest,2*M_PI);
    if (angle_error<0.01) {
        return TaskStatus::TASK_COMPLETED;
    }
    else if (angle_error>M_PI) {
        angle_error-=2*M_PI;
    }
    rot = ((angle_error>0)?+1:-1) * cfg.rotation_vel;

#ifdef DEBUG_GOTO
    ROS_INFO("Command vel %.2f angle_error %.2f rot %.2f\n",vel,angle_error,rot);
#endif

    env->publishVelocity(vel, rot);
	return TaskStatus::TASK_RUNNING;
}

TaskIndicator TaskRotate::terminate()
{
    env->publishVelocity(0,0);
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskRotate);
