#ifndef TASK_SETPTZ_H
#define TASK_SETPTZ_H

#include "task_manager_lib/TaskDefinition.h"
#include "sim_tasks/SimTasksEnv.h"
#include "sim_tasks/TaskSetPTZConfig.h"
#include "axis_camera/Axis.h"

using namespace task_manager_lib;

namespace sim_tasks {
    class TaskSetPTZ : public TaskInstance<TaskSetPTZConfig,SimTasksEnv>
    {

        protected:
            ros::Publisher axis_cmd;
            ros::Subscriber axis_state;
            axis_camera::Axis state;
            double init_time;

            void axisCallback(const axis_camera::AxisConstPtr & msg) {
                state = *msg;
            }
        public:
            TaskSetPTZ(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskSetPTZ() {};

            virtual TaskIndicator initialise(const TaskParameters & parameters) throw (InvalidParameter);

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
