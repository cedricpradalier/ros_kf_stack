#ifndef TASK_SET_HEADING_H
#define TASK_SET_HEADING_H

#include "task_manager_lib/TaskDefinition.h"
#include "sim_tasks/SimTasksEnv.h"
#include "sim_tasks_cfg/TaskSetHeadingConfig.h"

using namespace task_manager_lib;
using namespace sim_tasks_cfg;

namespace sim_tasks {
    class TaskSetHeading : public TaskInstance<TaskSetHeadingConfig,SimTasksEnv>
    {
        public:
            TaskSetHeading(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskSetHeading() {};

            virtual TaskIndicator initialise(const TaskParameters & parameters) ;

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();
    };
    class TaskFactorySetHeading : public TaskDefinition<TaskSetHeadingConfig, SimTasksEnv, TaskSetHeading>
    {

        public:
            TaskFactorySetHeading(TaskEnvironmentPtr env) : 
                Parent("SetHeading","Reach a desired heading angle",true,env) {}
            virtual ~TaskFactorySetHeading() {};
    };
};

#endif // TASK_SET_HEADING_H
