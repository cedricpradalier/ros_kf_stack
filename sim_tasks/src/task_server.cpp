#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <signal.h>

#include "task_manager_lib/TaskServerDefault.h"


#include "sim_tasks/SimTasksEnv.h"


using namespace sim_tasks;
using namespace task_manager_lib;


class TaskServer : public TaskServerBase {
    protected: 
    public:
        TaskServer(TaskEnvironmentPtr _env) : TaskServerBase(_env,true) {
            start();
        }

};

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"sim_tasks");//init ros
    ros::NodeHandle nh("~");
    TaskEnvironmentPtr env(new SimTasksEnv(nh));
    TaskServer ts(env);
    ros::spin();
    return 0;
}
