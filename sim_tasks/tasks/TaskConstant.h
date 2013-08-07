#ifndef TASK_CONSTANT_H
#define TASK_CONSTANT_H

#include <ros/ros.h>
#include "task_manager_lib/TaskDefinition.h"
#include "sim_tasks/SimTasksEnv.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "sim_tasks_cfg/TaskConstantConfig.h"

using namespace task_manager_lib;
using namespace sim_tasks_cfg;

namespace sim_tasks {
    class TaskConstant : public TaskInstance<TaskConstantConfig,SimTasksEnv>
    {

        protected:
            ros::Time initial_time;
        public:
            TaskConstant(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskConstant() {};

            virtual TaskIndicator initialise(const TaskParameters & parameters) ;

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();
    };
    class TaskFactoryConstant : public TaskDefinition<TaskConstantConfig, SimTasksEnv, TaskConstant>
    {

        public:
            TaskFactoryConstant(TaskEnvironmentPtr env) : 
                Parent("Constant","Apply a constant command for a given duration",true,env) {}
            virtual ~TaskFactoryConstant() {};
    };
};

#endif // TASK_CONSTANT_H
