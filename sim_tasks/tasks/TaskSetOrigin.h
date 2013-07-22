#ifndef TASK_SET_ORIGIN_H
#define TASK_SET_ORIGIN_H

#include "task_manager_lib/TaskDefinition.h"
#include "sim_tasks/SimTasksEnv.h"
#include "sim_tasks_cfg/TaskSetOriginConfig.h"

using namespace task_manager_lib;
using namespace sim_tasks_cfg;

namespace sim_tasks {
    class TaskSetOrigin : public TaskInstance<TaskSetOriginConfig,SimTasksEnv>
    {

        public:
            TaskSetOrigin(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskSetOrigin() {};

            virtual TaskIndicator iterate();

    };
    class TaskFactorySetOrigin : public TaskDefinition<TaskSetOriginConfig, SimTasksEnv, TaskSetOrigin>
    {

        public:
            TaskFactorySetOrigin(TaskEnvironmentPtr env) : 
                Parent("SetOrigin","Record an origin position for UTM coordinates",true,env) {}
            virtual ~TaskFactorySetOrigin() {};
    };
};

#endif // TASK_SET_ORIGIN_H
