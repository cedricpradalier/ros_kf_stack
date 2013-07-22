#include <math.h>
#include "TaskWaitForDistance.h"
#include "sim_tasks_cfg/TaskWaitForDistanceConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace sim_tasks_cfg;
using namespace sim_tasks;

TaskIndicator TaskWaitForDistance::initialise(const TaskParameters & parameters) 
{
    geometry_msgs::Pose2D tpose = env->getPose2D();
    start_x = tpose.x;
    start_y = tpose.y;
	return TaskStatus::TASK_INITIALISED;
}

TaskIndicator TaskWaitForDistance::iterate()
{
    geometry_msgs::Pose2D tpose = env->getPose2D();
    double r = hypot(start_y-tpose.y,start_x-tpose.x);
    if (r >= cfg.distance) {
        ROS_INFO("Crossed distance border at %.2f %.2f",tpose.x, tpose.y);
		return TaskStatus::TASK_COMPLETED;
    }
	return TaskStatus::TASK_RUNNING;
}

DYNAMIC_TASK(TaskFactoryWaitForDistance);
