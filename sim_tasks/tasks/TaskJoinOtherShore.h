#ifndef TASK_JOIN_OTHER_SHORE_H
#define TASK_JOIN_OTHER_SHORE_H

#include "task_manager_lib/TaskDefinition.h"
#include "sim_tasks/SimTasksEnv.h"
#include "sim_tasks/TaskJoinOtherShoreConfig.h"

using namespace task_manager_lib;

namespace sim_tasks {
    class TaskJoinOtherShore : public TaskDefinitionWithConfig<TaskJoinOtherShoreConfig, TaskJoinOtherShore>
    {

        protected:
            boost::shared_ptr<SimTasksEnv> env;
        public:
            TaskJoinOtherShore(boost::shared_ptr<TaskEnvironment> env); 
            virtual ~TaskJoinOtherShore() {};

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();
    };
};

#endif // TASK_JOIN_OTHER_SHORE_H
