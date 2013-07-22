#ifndef TASK_GOTO_H
#define TASK_GOTO_H

#include "task_manager_lib/TaskDefinition.h"
#include "sim_tasks/SimTasksEnv.h"
#include "sim_tasks_cfg/TaskGoToConfig.h"

using namespace task_manager_lib;
using namespace sim_tasks_cfg;

namespace sim_tasks {
    class TaskGoTo : public TaskInstance<TaskGoToConfig,SimTasksEnv>
    {
        protected:
            geometry_msgs::Pose2D tstart;
        public:
            TaskGoTo(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskGoTo() {};

            virtual TaskIndicator initialise(const TaskParameters & parameters) ;

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();
    };
    class TaskFactoryGoTo : public TaskDefinition<TaskGoToConfig, SimTasksEnv, TaskGoTo>
    {

        public:
            TaskFactoryGoTo(TaskEnvironmentPtr env) : 
                Parent("GoTo","Reach a desired destination",true,env) {}
            virtual ~TaskFactoryGoTo() {};
    };
};

#endif // TASK_GOTO_H
