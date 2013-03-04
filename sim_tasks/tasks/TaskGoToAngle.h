#ifndef TASK_GOTO_ANGLE_H
#define TASK_GOTO_ANGLE_H

#include "task_manager_lib/TaskDefinition.h"
#include "sim_tasks/SimTasksEnv.h"
#include "sim_tasks/TaskGoToAngleConfig.h"

using namespace task_manager_lib;

namespace sim_tasks {
    class TaskGoToAngle : public TaskDefinitionWithConfig<TaskGoToAngleConfig, TaskGoToAngle>
    {

        protected:
            boost::shared_ptr<SimTasksEnv> env;
        public:
            TaskGoToAngle(boost::shared_ptr<TaskEnvironment> env); 
            virtual ~TaskGoToAngle() {};

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();
    };
};

#endif // TASK_GOTO_H
