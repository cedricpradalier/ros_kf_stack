#ifndef TASK_SET_MANUAL_H
#define TASK_SET_MANUAL_H

#include "task_manager_lib/TaskDefinition.h"
#include "sim_tasks/SimTasksEnv.h"
#include "sim_tasks/TaskSetManualConfig.h"

using namespace task_manager_lib;

namespace sim_tasks {
    class TaskSetManual : public TaskInstance<TaskSetManualConfig,SimTasksEnv>
    {
        public:
            TaskSetManual(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskSetManual() {};

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();
    };
    class TaskFactorySetManual : public TaskDefinition<TaskSetManualConfig, SimTasksEnv, TaskSetManual>
    {

        public:
            TaskFactorySetManual(TaskEnvironmentPtr env) : 
                Parent("SetManual","Set the control mux to manual mode",false,env) {}
            virtual ~TaskFactorySetManual() {};
    };
};

#endif // TASK_SET_MANUAL_H
