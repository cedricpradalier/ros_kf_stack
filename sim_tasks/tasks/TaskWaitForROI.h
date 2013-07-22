#ifndef TASK_WAIT_FOR_ROI_H
#define TASK_WAIT_FOR_ROI_H

#include "task_manager_lib/TaskDefinition.h"
#include "sim_tasks/SimTasksEnv.h"
#include "sim_tasks_cfg/TaskWaitForROIConfig.h"

using namespace task_manager_lib;
using namespace sim_tasks_cfg;

namespace sim_tasks {
    class TaskWaitForROI : public TaskInstance<TaskWaitForROIConfig,SimTasksEnv>
    {
        protected:
            bool exited;
            // We need to store these because they can be initialised with
            // current position. If we were modifying cfg.roi_x/y then this could be
            // modified with weird effect in dynamic_reconfigure
            float roi_x, roi_y;
        public:
            TaskWaitForROI(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskWaitForROI() {};

            virtual TaskIndicator initialise(const TaskParameters & parameters) ;

            virtual TaskIndicator iterate();

    };
    class TaskFactoryWaitForROI : public TaskDefinition<TaskWaitForROIConfig, SimTasksEnv, TaskWaitForROI>
    {
        public:
            TaskFactoryWaitForROI(TaskEnvironmentPtr env) : 
                Parent("WaitForROI","Do nothing until we reach a given destination",true,env) {}
            virtual ~TaskFactoryWaitForROI() {};
    };
};

#endif // TASK_WAIT_FOR_ROI_H
