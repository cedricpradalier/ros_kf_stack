#include <math.h>
#include "TaskGoToAngle.h"
#include "sim_tasks/TaskGoToAngleConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace sim_tasks;

// #define DEBUG_GOTO

TaskGoToAngle::TaskGoToAngle(boost::shared_ptr<TaskEnvironment> tenv) 
    : TaskDefinitionWithConfig<TaskGoToAngleConfig,TaskGoToAngle>("GoToAngle","Reach a desired destination and angle",true,-1.)
{
    env = boost::dynamic_pointer_cast<SimTasksEnv,TaskEnvironment>(tenv);
}

TaskIndicator TaskGoToAngle::iterate()
{
    const geometry_msgs::Pose2D & tpose = env->getPose2D();
    double r = hypot(cfg.goal_y-tpose.y,cfg.goal_x-tpose.x);
    if ((r < cfg.dist_threshold) && (fabs(cfg.goal_theta - tpose.theta) < cfg.angle_threshold)) {
		return TaskStatus::TASK_COMPLETED;
    }
    double alpha = remainder(atan2(cfg.goal_y-tpose.y,cfg.goal_x-tpose.x)-tpose.theta,2*M_PI);
    double beta = remainder(-alpha-tpose.theta+cfg.goal_theta,2*M_PI);
#ifdef DEBUG_GOTO
    printf("c %.1f %.1f %.1f g %.1f %.1f r %.3f alpha %.1f beta %.1f\n",
            tpose.x, tpose.y, tpose.theta*180./M_PI,
            cfg.goal_x,cfg.goal_y,r,alpha*180./M_PI,beta*180./M_PI);
#endif
    if (fabs(alpha) >= M_PI/2) {
        double rot = ((alpha>0)?+1:-1)*M_PI/3;
#ifdef DEBUG_GOTO
        printf("Cmd v %.2f r %.2f\n",0.,rot);
#endif
        env->publishVelocity(0,rot);
    } else {
        double vel = cfg.k_v * r;
        double rot = cfg.k_alpha*alpha + cfg.k_beta*beta;
        if (vel > cfg.max_velocity) vel = cfg.max_velocity;
#ifdef DEBUG_GOTO
        printf("Cmd v %.2f r %.2f\n",vel,rot);
#endif
        env->publishVelocity(vel, rot);
    }
	return TaskStatus::TASK_RUNNING;
}

TaskIndicator TaskGoToAngle::terminate()
{
    env->publishVelocity(0,0);
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskGoToAngle);
