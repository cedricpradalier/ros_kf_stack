#include <math.h>
#include "TaskSetManual.h"
#include "sim_tasks_cfg/TaskSetManualConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace sim_tasks_cfg;
using namespace sim_tasks;


TaskIndicator TaskSetManual::iterate()
{
    boost::lock_guard<boost::mutex> guard(env->getMutex());
    env->setManualControl();
	return TaskStatus::TASK_COMPLETED;
}

TaskIndicator TaskSetManual::terminate()
{
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskFactorySetManual);
