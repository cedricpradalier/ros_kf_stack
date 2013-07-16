#include <math.h>
#include "TaskSetPTZ.h"
#include "sim_tasks/TaskSetPTZConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace sim_tasks;

// #define DEBUG_SETPTZ


TaskIndicator TaskSetPTZ::initialise(const TaskParameters & parameters) 
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
    last_publish_time = init_time = ros::Time::now().toSec();
    if (cfg.max_command_rate>0.) {
        last_publish_time -= 2.0/cfg.max_command_rate;
    }

    ROS_INFO("Setting PTZ to %.2f %.2f %.2f",cfg.pan,cfg.tilt,cfg.zoom);
    return ti;
}


TaskIndicator TaskSetPTZ::iterate()
{
    axis_camera::Axis cmd;
    double now = ros::Time::now().toSec();
    cmd.pan = cfg.pan * 180./M_PI;
    cmd.tilt = cfg.tilt * 180./M_PI;
    cmd.zoom = cfg.zoom;
    if ((cfg.max_command_rate==0.0) || (now - last_publish_time > 1.0/cfg.max_command_rate)) {
        ROS_INFO("Publishing PTZ");
        axis_cmd.publish(cmd);
        last_publish_time = now;
    }
    if (cfg.wait_timeout<=0) {
        return TaskStatus::TASK_COMPLETED;
    }
    if ((fabs(remainder(cmd.pan - state.pan,360.0))<1) && (fabs(remainder(cmd.tilt - state.tilt,360.0))<1) 
            && (fabs(cmd.zoom - state.zoom)<1)) {
        return TaskStatus::TASK_COMPLETED;
    }
    if ((now - init_time) > cfg.wait_timeout) {
        return TaskStatus::TASK_TIMEOUT;
    }
	return TaskStatus::TASK_RUNNING;
}

TaskIndicator TaskSetPTZ::terminate()
{
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskFactorySetPTZ);
