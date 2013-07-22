#ifndef TASK_ALIGN_WITH_SHORE_H
#define TASK_ALIGN_WITH_SHORE_H

#include "task_manager_lib/TaskDefinition.h"
#include "sim_tasks/SimTasksEnv.h"
#include "sim_tasks_cfg/TaskAlignWithShoreConfig.h"

using namespace task_manager_lib;
using namespace sim_tasks_cfg;

namespace sim_tasks {
    class TaskAlignWithShore : public TaskInstance<TaskAlignWithShoreConfig,SimTasksEnv>
    {

        public:
            TaskAlignWithShore(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskAlignWithShore() {};

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();

    };
    class TaskFactoryAlignWithShore : public TaskDefinition<TaskAlignWithShoreConfig, SimTasksEnv, TaskAlignWithShore>
    {

        public:
            TaskFactoryAlignWithShore(TaskEnvironmentPtr env) : 
                Parent("AlignWithShore","Rotate on the spot until the closest point (the shore) is at the target bearing",true,env) {}
            virtual ~TaskFactoryAlignWithShore() {};
    };
};

#endif // TASK_ALIGN_WITH_SHORE_H
