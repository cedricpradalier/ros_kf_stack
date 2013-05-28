#ifndef TASK_FOLLOW_SHORE_PID_H
#define TASK_FOLLOW_SHORE_PID_H

#include "task_manager_lib/TaskDefinition.h"
#include "sim_tasks/SimTasksEnv.h"
#include "sim_tasks/TaskFollowShorePIDConfig.h"

using namespace task_manager_lib;

namespace sim_tasks {
    class TaskFollowShorePID : public TaskDefinitionWithConfig<TaskFollowShorePIDConfig, TaskFollowShorePID>
    {

        protected:
            boost::shared_ptr<SimTasksEnv> env;
            bool outOfStartBox;
            bool backToStartBox;
        public:
            TaskFollowShorePID(boost::shared_ptr<TaskEnvironment> env); 
            virtual ~TaskFollowShorePID() {};

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();
    };
};

#endif // TASK_FOLLOW_SHORE_PID_H
