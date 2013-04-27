#ifndef TASK_ALIGN_WITH_SHORE_H
#define TASK_ALIGN_WITH_SHORE_H

#include "task_manager_lib/TaskDefinition.h"
#include "sim_tasks/SimTasksEnv.h"
#include "sim_tasks/TaskAlignWithShoreConfig.h"

using namespace task_manager_lib;

namespace sim_tasks {
    class TaskAlignWithShore : public TaskDefinitionWithConfig<TaskAlignWithShoreConfig, TaskAlignWithShore>
    {

        protected:
            boost::shared_ptr<SimTasksEnv> env;
        public:
            TaskAlignWithShore(boost::shared_ptr<TaskEnvironment> env); 
            virtual ~TaskAlignWithShore() {};

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();

    };
};

#endif // TASK_ALIGN_WITH_SHORE_H
