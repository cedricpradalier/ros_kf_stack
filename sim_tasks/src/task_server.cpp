#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <signal.h>

#include "task_manager_lib/SequenceTask.h"
#include "task_manager_lib/TaskScheduler.h"

#include "task_manager_lib/DynamicTask.h"
#include "task_manager_lib/TaskIdleDefault.h"
#include "task_manager_lib/TaskWaitDefault.h"

// #include "task_manager_lib/TaskServerInterface.h"


#include "sim_tasks/SimTasksEnv.h"


using namespace sim_tasks;
using namespace task_manager_lib;

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"sim_tasks");//init ros
    ros::NodeHandle nh("~");//initialize node
    std::string lib_path = "./lib";
    nh.getParam("lib_path",lib_path);

    boost::shared_ptr<TaskEnvironment> env(new SimTasksEnv(nh));
    boost::shared_ptr<TaskDefinition> idle(new TaskIdleDefault(env));
    boost::shared_ptr<TaskDefinition> wait(new TaskWaitDefault(env));
    TaskScheduler ts(nh, idle, 0.5);
    // TaskServerInterface tsi(ts);
    ts.addTask(wait);
    ts.loadAllTasks(lib_path,env);
    ts.configureTasks();
    ts.printTaskDirectory(true);
    ts.startScheduler();
    ros::spin();
    return 0;
}
