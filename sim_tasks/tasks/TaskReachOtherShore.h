#ifndef TASK_REACH_OTHER_SHORE_H
#define TASK_REACH_OTHER_SHORE_H

#include "task_manager_lib/TaskDefinition.h"
#include "sim_tasks/SimTasksEnv.h"
#include "sim_tasks/TaskReachOtherShoreConfig.h"

using namespace task_manager_lib;

namespace sim_tasks {
    class TaskReachOtherShore : public TaskInstance<TaskReachOtherShoreConfig,SimTasksEnv>
    {
        public:
            TaskReachOtherShore(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskReachOtherShore() {};

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();
    };
    class TaskFactoryReachOtherShore : public TaskDefinition<TaskReachOtherShoreConfig, SimTasksEnv, TaskReachOtherShore>
    {

        public:
            TaskFactoryReachOtherShore(TaskEnvironmentPtr env) : 
                Parent("ReachOtherShore","Rotate and reach the other shore",true,env) {}
            virtual ~TaskFactoryReachOtherShore() {};
    };
};

#endif // TASK_REACH_OTHER_SHORE_H
