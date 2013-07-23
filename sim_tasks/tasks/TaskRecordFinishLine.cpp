#include <math.h>
#include "TaskRecordFinishLine.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace sim_tasks;

// #define DEBUG_GOTO


TaskIndicator TaskRecordFinishLine::iterate()
{
    const geometry_msgs::Pose2D & tpose = env->getPose2D();
    env->setFinishLine2D(tpose);
    ROS_INFO("finishLine: x %.3f - y %.3f - theta %.3f",tpose.x,tpose.y,tpose.theta);
    return TaskStatus::TASK_COMPLETED;
}

DYNAMIC_TASK(TaskFactoryRecordFinishLine);
