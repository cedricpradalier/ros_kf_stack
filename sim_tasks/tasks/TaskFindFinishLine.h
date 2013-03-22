#ifndef TASK_FIND_FINISH_LINE_H
#define TASK_FIND_FINISH_LINE_H

#include "task_manager_lib/TaskDefinition.h"
#include "sim_tasks/SimTasksEnv.h"
#include "sim_tasks/TaskFindFinishLineConfig.h"

using namespace task_manager_lib;

namespace sim_tasks {
    class TaskFindFinishLine : public TaskDefinitionWithConfig<TaskFindFinishLineConfig, TaskFindFinishLine>
    {

        protected:
            boost::shared_ptr<SimTasksEnv> env;
        public:
            TaskFindFinishLine(boost::shared_ptr<TaskEnvironment> env); 
            virtual ~TaskFindFinishLine() {};

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();
    };
};

#endif // TASK_FIND_FINISH_LINE_H
