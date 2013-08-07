
#include <math.h>
#include "TaskConstant.h"
#include "sim_tasks_cfg/TaskConstantConfig.h"

using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace sim_tasks_cfg;
using namespace sim_tasks;

// #define DEBUG_GOTO

TaskIndicator TaskConstant::initialise(const TaskParameters & parameters) 
{
    initial_time = ros::Time::now();
    return TaskStatus::TASK_INITIALISED;
}


TaskIndicator TaskConstant::iterate()
{
    if ((ros::Time::now() - initial_time).toSec() > cfg.duration) {
        return TaskStatus::TASK_COMPLETED;
    }
    env->publishVelocity(cfg.linear, cfg.angular);
	return TaskStatus::TASK_RUNNING;
}

TaskIndicator TaskConstant::terminate()
{
    env->publishVelocity(0,0);
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskFactoryConstant);
