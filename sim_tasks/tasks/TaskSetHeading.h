#ifndef TASK_SET_HEADING_H
#define TASK_SET_HEADING_H

#include "task_manager_lib/TaskDefinition.h"
#include "sim_tasks/SimTasksEnv.h"
#include "sim_tasks/TaskSetHeadingConfig.h"

using namespace task_manager_lib;

namespace sim_tasks {
    class TaskSetHeading : public TaskDefinitionWithConfig<TaskSetHeadingConfig, TaskSetHeading>
    {

        protected:
            boost::shared_ptr<SimTasksEnv> env;
        public:
            TaskSetHeading(boost::shared_ptr<TaskEnvironment> env); 
            virtual ~TaskSetHeading() {};

            virtual TaskIndicator initialise(const TaskParameters & parameters) throw (InvalidParameter);

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();
    };
};

#endif // TASK_SET_HEADING_H
