#include <math.h>
#include "TaskGoTo.h"
#include "sim_tasks_cfg/TaskGoToConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace sim_tasks_cfg;
using namespace sim_tasks;

// #define DEBUG_GOTO
#ifdef DEBUG_GOTO
#warning Debugging task GOTO
#endif


TaskIndicator TaskGoTo::initialise(const TaskParameters & parameters) 
{
    if (cfg.relative) {
        tstart = env->getPose2D(cfg.wrtOrigin);
    }
    ROS_INFO("Going to %.2f %.2f",cfg.goal_x,cfg.goal_y);
    return TaskStatus::TASK_INITIALISED;
}


TaskIndicator TaskGoTo::iterate()
{
    geometry_msgs::Pose2D tpose = env->getPose2D(cfg.wrtOrigin);
    if (cfg.relative) { 
        tpose.x -= tstart.x;
        tpose.y -= tstart.y;
    }
    double r = hypot(cfg.goal_y-tpose.y,cfg.goal_x-tpose.x);
    if (r < cfg.dist_threshold) {
        env->publishVelocity(0,0);
		return TaskStatus::TASK_COMPLETED;
    }
    double alpha = remainder(atan2((cfg.goal_y-tpose.y),cfg.goal_x-tpose.x)-tpose.theta,2*M_PI);
#ifdef DEBUG_GOTO
    printf("c %.1f %.1f %.1f g %.1f %.1f r %.3f alpha %.1f\n",
            tpose.x, tpose.y, tpose.theta*180./M_PI,
            cfg.goal_x,cfg.goal_y,r,alpha*180./M_PI);
#endif
    if (fabs(alpha) > M_PI/6) {
        int ska = (cfg.k_alpha>=0)?+1:-1;
        double rot = ((alpha>0)?-ska:ska)*M_PI/3;
#ifdef DEBUG_GOTO
        printf("Cmd v %.2f r %.2f\n",0.,rot);
#endif
        env->publishVelocity(0,rot);
    } else {
        double vel = cfg.k_v * r;
        double rot = -cfg.k_alpha*alpha;
        if (vel > cfg.max_velocity) vel = cfg.max_velocity;
#ifdef DEBUG_GOTO
        printf("Cmd v %.2f r %.2f\n",vel,rot);
#endif
        env->publishVelocity(vel, rot);
    }
	return TaskStatus::TASK_RUNNING;
}

TaskIndicator TaskGoTo::terminate()
{
    env->publishVelocity(0,0);
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskFactoryGoTo);
