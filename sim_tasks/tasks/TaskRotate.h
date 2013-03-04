#ifndef TASK_ROTATE_H
#define TASK_ROTATE_H

#include "task_manager_lib/TaskDefinition.h"
#include "sim_tasks/SimTasksEnv.h"
#include "sim_tasks/TaskRotateConfig.h"

using namespace task_manager_lib;

namespace sim_tasks {
    class TaskRotate : public TaskDefinitionWithConfig<TaskRotateConfig, TaskRotate>
    {

        protected:
            boost::shared_ptr<SimTasksEnv> env;
        public:
            TaskRotate(boost::shared_ptr<TaskEnvironment> env); 
            virtual ~TaskRotate() {};

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();
    };
};

#endif // TASK_ROTATE_H
