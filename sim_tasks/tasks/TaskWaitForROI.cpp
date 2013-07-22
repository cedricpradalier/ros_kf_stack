#include <math.h>
#include "TaskWaitForROI.h"
#include "sim_tasks_cfg/TaskWaitForROIConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace sim_tasks_cfg;
using namespace sim_tasks;

TaskIndicator TaskWaitForROI::initialise(const TaskParameters & parameters) 
{
    geometry_msgs::Pose2D tpose = env->getPose2D(cfg.wrtOrigin);
    roi_x = cfg.roi_x;
    roi_y = cfg.roi_y;
    if (cfg.current) {
        roi_x = tpose.x;
        roi_y = tpose.y;
    }
    exited = (cfg.histeresis_radius < 0); 
	return TaskStatus::TASK_INITIALISED;
}

TaskIndicator TaskWaitForROI::iterate()
{
    geometry_msgs::Pose2D tpose = env->getPose2D(cfg.wrtOrigin);
    double r = hypot(roi_y-tpose.y,roi_x-tpose.x);
    if (!exited && (cfg.histeresis_radius >= 0)) { 
        exited = r >= (cfg.roi_radius + cfg.histeresis_radius);
    }
    if (exited && (r < cfg.roi_radius)) {
        ROS_INFO("Detected ROI at %.2f %.2f",tpose.x, tpose.y);
		return TaskStatus::TASK_COMPLETED;
    }
	return TaskStatus::TASK_RUNNING;
}

DYNAMIC_TASK(TaskFactoryWaitForROI);
