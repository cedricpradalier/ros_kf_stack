#ifndef TASK_FOLLOW_SHORE_PID_H
#define TASK_FOLLOW_SHORE_PID_H

#include "task_manager_lib/TaskDefinition.h"
#include "sim_tasks/SimTasksEnv.h"
#include "sim_tasks_cfg/TaskFollowShorePIDConfig.h"

using namespace task_manager_lib;
using namespace sim_tasks_cfg;

namespace sim_tasks {
    class TaskFollowShorePID : public TaskInstance<TaskFollowShorePIDConfig,SimTasksEnv>
    {

        protected:
            bool outOfStartBox;
            bool backToStartBox;
            ros::Publisher status_dist_pub;
            ros::Publisher status_angle_pub;
            tf::TransformBroadcaster br;
            float angle_error_prev;
            float d_angle_error;
            float i_angle_error;
            float distance_error_prev;
            float d_distance_error;
            float i_distance_error;

            float saturate(float value, float max) {
                if (value > max) return max;
                if (value < -max) return -max;
                return value;
            }

            float saturate(float value, float min, float max) {
                if (value > max) return max;
                if (value < -min) return -min;
                return value;
            }

        public:
            TaskFollowShorePID(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskFollowShorePID() {};

            virtual TaskIndicator initialise(const TaskParameters & parameters) ;

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();
    };
    class TaskFactoryFollowShorePID : public TaskDefinition<TaskFollowShorePIDConfig, SimTasksEnv, TaskFollowShorePID>
    {

        public:
            TaskFactoryFollowShorePID(TaskEnvironmentPtr env) : 
                Parent("FollowShorePID","Follow the shore of the lake with PID controller",true,env) {}
            virtual ~TaskFactoryFollowShorePID() {};
    };
};

#endif // TASK_FOLLOW_SHORE_PID_H
