#ifndef TASK_GOTO_ANGLE_H
#define TASK_GOTO_ANGLE_H

#include "task_manager_lib/TaskDefinition.h"
#include "sim_tasks/SimTasksEnv.h"
#include "sim_tasks_cfg/TaskGoToAngleConfig.h"

using namespace task_manager_lib;
using namespace sim_tasks_cfg;

namespace sim_tasks {
    class TaskGoToAngle : public TaskInstance<TaskGoToAngleConfig,SimTasksEnv>
    {
        public:
            TaskGoToAngle(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskGoToAngle() {};

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();
    };
    class TaskFactoryGoToAngle : public TaskDefinition<TaskGoToAngleConfig, SimTasksEnv, TaskGoToAngle>
    {

        public:
            TaskFactoryGoToAngle(TaskEnvironmentPtr env) : 
                Parent("GoToAngle","Reach a desired destination and angle",true,env) {}
            virtual ~TaskFactoryGoToAngle() {};
    };
};

#endif // TASK_GOTO_H
