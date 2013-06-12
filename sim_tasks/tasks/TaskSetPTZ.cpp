#include <math.h>
#include "TaskSetPTZ.h"
#include "sim_tasks/TaskSetPTZConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace sim_tasks;

// #define DEBUG_GOTO

TaskSetPTZ::TaskSetPTZ(boost::shared_ptr<TaskEnvironment> tenv) 
    : TaskDefinitionWithConfig<TaskSetPTZConfig,TaskSetPTZ>("SetPTZ","Reach a PTZ configuration",true,-1.)
{
    env = boost::dynamic_pointer_cast<SimTasksEnv,TaskEnvironment>(tenv);
}

TaskIndicator TaskSetPTZ::initialise(const TaskParameters & parameters) throw (InvalidParameter)
{
    TaskIndicator ti = Parent::initialise(parameters);
    if (ti != TaskStatus::TASK_INITIALISED) {
        return ti;
    }
    if (cfg.zoom < 1) {
        cfg.zoom = 1;
    }
    state.zoom = -1;
    axis_cmd = env->getNodeHandle().advertise<axis_camera::Axis>("/axis/cmd",1);
    axis_state = env->getNodeHandle().subscribe("/axis/state",1,&TaskSetPTZ::axisCallback,this);
    init_time = ros::Time::now().toSec();

    ROS_INFO("Setting PTZ to %.2f %.2f %.2f",cfg.pan,cfg.tilt,cfg.zoom);
    return ti;
}


TaskIndicator TaskSetPTZ::iterate()
{
    axis_camera::Axis cmd;
    cmd.pan = cfg.pan * 180./M_PI;
    cmd.tilt = cfg.tilt * 180./M_PI;
    cmd.zoom = cfg.zoom;
    axis_cmd.publish(cmd);
    if (cfg.wait_timeout<=0) {
        return TaskStatus::TASK_COMPLETED;
    }
    if ((fabs(cmd.pan - state.pan)<1) && (fabs(cmd.tilt - state.tilt)<1) 
            && (fabs(cmd.zoom - state.zoom)<1)) {
        return TaskStatus::TASK_COMPLETED;
    }
    if ((ros::Time::now().toSec() - init_time) > cfg.wait_timeout) {
        return TaskStatus::TASK_TIMEOUT;
    }
	return TaskStatus::TASK_RUNNING;
}

TaskIndicator TaskSetPTZ::terminate()
{
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskSetPTZ);
