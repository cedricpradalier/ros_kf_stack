#ifndef TASK_GOTO_H
#define TASK_GOTO_H

#include "task_manager_lib/TaskDefinition.h"
#include "sim_tasks/SimTasksEnv.h"
#include "sim_tasks/TaskGoToConfig.h"

using namespace task_manager_lib;

namespace sim_tasks {
    class TaskGoTo : public TaskDefinitionWithConfig<TaskGoToConfig, TaskGoTo>
    {

        protected:
            boost::shared_ptr<SimTasksEnv> env;
        public:
            TaskGoTo(boost::shared_ptr<TaskEnvironment> env); 
            virtual ~TaskGoTo() {};

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();
    };
};

#endif // TASK_GOTO_H
