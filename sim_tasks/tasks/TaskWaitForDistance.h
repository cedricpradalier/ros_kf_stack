#ifndef TASK_WAIT_FOR_DISTANCE_H
#define TASK_WAIT_FOR_DISTANCE_H

#include "task_manager_lib/TaskDefinition.h"
#include "sim_tasks/SimTasksEnv.h"
#include "sim_tasks_cfg/TaskWaitForDistanceConfig.h"

using namespace task_manager_lib;
using namespace sim_tasks_cfg;

namespace sim_tasks {
    class TaskWaitForDistance : public TaskInstance<TaskWaitForDistanceConfig,SimTasksEnv>
    {
        protected:
            float start_x, start_y;
        public:
            TaskWaitForDistance(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskWaitForDistance() {};

            virtual TaskIndicator initialise(const TaskParameters & parameters) ;

            virtual TaskIndicator iterate();

    };
    class TaskFactoryWaitForDistance : public TaskDefinition<TaskWaitForDistanceConfig, SimTasksEnv, TaskWaitForDistance>
    {
        public:
            TaskFactoryWaitForDistance(TaskEnvironmentPtr env) : 
                Parent("WaitForDistance","Do nothing until we reach a given distance from start point",true,env) {}
            virtual ~TaskFactoryWaitForDistance() {};
    };
};

#endif // TASK_WAIT_FOR_DISTANCE_H
