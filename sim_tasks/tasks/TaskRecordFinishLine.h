#ifndef TASK_RECORD_FINISH_LINE_H
#define TASK_RECORD_FINISH_LINE_H

#include "task_manager_lib/TaskDefinition.h"
#include "sim_tasks/SimTasksEnv.h"
#include "task_manager_lib/MinimalTaskConfig.h"

using namespace task_manager_lib;

namespace sim_tasks {
    class TaskRecordFinishLine : public TaskInstance<MinimalTaskConfig,SimTasksEnv>
    {
        public:
            TaskRecordFinishLine(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskRecordFinishLine() {};

            virtual TaskIndicator iterate();
    };
    class TaskFactoryRecordFinishLine : public TaskDefinition<MinimalTaskConfig, SimTasksEnv, TaskRecordFinishLine>
    {

        public:
            TaskFactoryRecordFinishLine(TaskEnvironmentPtr env) : 
                Parent("RecordFinishLine","Record the pose as finish line",true,env) {}
            virtual ~TaskFactoryRecordFinishLine() {};
    };
};

#endif // TASK_RECORD_FINISH_LINE_H
