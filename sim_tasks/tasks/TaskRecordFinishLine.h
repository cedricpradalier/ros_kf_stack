#ifndef TASK_RECORD_FINISH_LINE_H
#define TASK_RECORD_FINISH_LINE_H

#include "task_manager_lib/TaskDefinition.h"
#include "sim_tasks/SimTasksEnv.h"
#include "task_manager_lib/MinimalTaskConfig.h"

using namespace task_manager_lib;

namespace sim_tasks {
    class TaskRecordFinishLine : public TaskDefinitionWithConfig<MinimalTaskConfig, TaskRecordFinishLine>
    {

        protected:
            boost::shared_ptr<SimTasksEnv> env;
        public:
            TaskRecordFinishLine(boost::shared_ptr<TaskEnvironment> env); 
            virtual ~TaskRecordFinishLine() {};

            virtual TaskIndicator iterate();

            virtual boost::shared_ptr<TaskDefinition> getInstance() {
                return shared_from_this();
            }
    };
};

#endif // TASK_RECORD_FINISH_LINE_H
