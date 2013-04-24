#ifndef TASK_GOTO_H
#define TASK_GOTO_H

#include "task_manager_lib/TaskDefinition.h"
#include "sim_tasks/SimTasksEnv.h"
#include "sim_tasks/TaskSetPTZConfig.h"
#include "axis_camera/Axis.h"

using namespace task_manager_lib;

namespace sim_tasks {
    class TaskSetPTZ : public TaskDefinitionWithConfig<TaskSetPTZConfig, TaskSetPTZ>
    {

        protected:
            boost::shared_ptr<SimTasksEnv> env;
            ros::Publisher axis_cmd;
            ros::Subscriber axis_state;
            axis_camera::Axis state;
            double init_time;

            void axisCallback(const axis_camera::AxisConstPtr & msg) {
                state = *msg;
            }
        public:
            TaskSetPTZ(boost::shared_ptr<TaskEnvironment> env); 
            virtual ~TaskSetPTZ() {};

            virtual TaskIndicator initialise(const TaskParameters & parameters) throw (InvalidParameter);

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();
    };
};

#endif // TASK_GOTO_H
