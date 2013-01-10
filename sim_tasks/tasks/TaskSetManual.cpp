#include <math.h>
#include "TaskSetManual.h"
#include "sim_tasks/TaskSetManualConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace sim_tasks;

TaskSetManual::TaskSetManual(boost::shared_ptr<TaskEnvironment> tenv) 
    : TaskDefinitionWithConfig<TaskSetManualConfig,TaskSetManual>("SetManual","Set the control mux to manual mode",false,-1.)
{
    env = boost::dynamic_pointer_cast<SimTasksEnv,TaskEnvironment>(tenv);
}

TaskIndicator TaskSetManual::iterate()
{
    env->setManualControl();
	return TaskStatus::TASK_COMPLETED;
}

TaskIndicator TaskSetManual::terminate()
{
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskSetManual);
