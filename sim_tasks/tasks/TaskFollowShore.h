#ifndef TASK_FOLLOW_SHORE_H
#define TASK_FOLLOW_SHORE_H

#include "task_manager_lib/TaskDefinition.h"
#include "sim_tasks/SimTasksEnv.h"
#include "sim_tasks_cfg/TaskFollowShoreConfig.h"

using namespace task_manager_lib;
using namespace sim_tasks_cfg;

namespace sim_tasks {
    class TaskFollowShore : public TaskInstance<TaskFollowShoreConfig,SimTasksEnv>
    {

        protected:
            bool outOfStartBox;
            bool backToStartBox;
            ros::Publisher status_pub;
        public:
            TaskFollowShore(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskFollowShore() {};

            virtual TaskIndicator initialise(const TaskParameters & parameters) ;

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();
    };
    class TaskFactoryFollowShore : public TaskDefinition<TaskFollowShoreConfig, SimTasksEnv, TaskFollowShore>
    {

        public:
            TaskFactoryFollowShore(TaskEnvironmentPtr env) : 
                Parent("FollowShore","Follow the shore of the lake",true,env) {}
            virtual ~TaskFactoryFollowShore() {};
    };
};

#endif // TASK_FOLLOW_SHORE_H
