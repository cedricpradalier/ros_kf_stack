#ifndef TASK_SETPTZ_H
#define TASK_SETPTZ_H

#include "task_manager_lib/TaskDefinition.h"
#include "sim_tasks/SimTasksEnv.h"
#include "sim_tasks_cfg/TaskSetPTZConfig.h"
#include "axis_camera/Axis.h"

using namespace task_manager_lib;
using namespace sim_tasks_cfg;

namespace sim_tasks {
    class TaskSetPTZ : public TaskInstance<TaskSetPTZConfig,SimTasksEnv>
    {

        protected:
            double init_time, last_publish_time;

        public:
            TaskSetPTZ(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskSetPTZ() {};

            virtual TaskIndicator initialise(const TaskParameters & parameters) ;

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();
    };
    class TaskFactorySetPTZ : public TaskDefinition<TaskSetPTZConfig, SimTasksEnv, TaskSetPTZ>
    {

        public:
            TaskFactorySetPTZ(TaskEnvironmentPtr env) : 
                Parent("SetPTZ","Reach a PTZ configuration",true,env) {}
            virtual ~TaskFactorySetPTZ() {};
    };
};

#endif // TASK_SETPTZ_H
