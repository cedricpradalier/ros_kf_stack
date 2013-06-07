#ifndef TASK_FOLLOW_SHORE_H
#define TASK_FOLLOW_SHORE_H

#include "task_manager_lib/TaskDefinition.h"
#include "sim_tasks/SimTasksEnv.h"
#include "sim_tasks/TaskFollowShoreConfig.h"

using namespace task_manager_lib;

namespace sim_tasks {
    class TaskFollowShore : public TaskDefinitionWithConfig<TaskFollowShoreConfig, TaskFollowShore>
    {

        protected:
            boost::shared_ptr<SimTasksEnv> env;
            bool outOfStartBox;
            bool backToStartBox;
            ros::Publisher status_pub;
        public:
            TaskFollowShore(boost::shared_ptr<TaskEnvironment> env); 
            virtual ~TaskFollowShore() {};

            virtual TaskIndicator initialise(const TaskParameters & parameters) throw (InvalidParameter);

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();
    };
};

#endif // TASK_FOLLOW_SHORE_H
