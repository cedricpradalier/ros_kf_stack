#ifndef TASK_REACH_OTHER_SHORE_H
#define TASK_REACH_OTHER_SHORE_H

#include "task_manager_lib/TaskDefinition.h"
#include "sim_tasks/SimTasksEnv.h"
#include "sim_tasks/TaskReachOtherShoreConfig.h"

using namespace task_manager_lib;

namespace sim_tasks {
    class TaskReachOtherShore : public TaskDefinitionWithConfig<TaskReachOtherShoreConfig, TaskReachOtherShore>
    {

        protected:
            boost::shared_ptr<SimTasksEnv> env;
        public:
            TaskReachOtherShore(boost::shared_ptr<TaskEnvironment> env); 
            virtual ~TaskReachOtherShore() {};

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();
    };
};

#endif // TASK_REACH_OTHER_SHORE_H
