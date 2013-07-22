#ifndef TASK_WAIT4AUTO_H
#define TASK_WAIT4AUTO_H

#include "task_manager_lib/TaskDefinition.h"
#include "sim_tasks/SimTasksEnv.h"
#include "sim_tasks_cfg/TaskWaitForAutoConfig.h"

using namespace task_manager_lib;
using namespace sim_tasks_cfg;

namespace sim_tasks {
    class TaskWaitForAuto : public TaskInstance<TaskWaitForAutoConfig,SimTasksEnv>
    {
        public:
            TaskWaitForAuto(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskWaitForAuto() {};

            virtual TaskIndicator initialise(const TaskParameters & parameters) ;

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();
    };
    class TaskFactoryWaitForAuto : public TaskDefinition<TaskWaitForAutoConfig, SimTasksEnv, TaskWaitForAuto>
    {

        public:
            TaskFactoryWaitForAuto(TaskEnvironmentPtr env) : 
                Parent("WaitForAuto","Wait for the control mux to switch to auto",true,env) {}
            virtual ~TaskFactoryWaitForAuto() {};
    };
};

#endif // TASK_WAIT4AUTO_H
