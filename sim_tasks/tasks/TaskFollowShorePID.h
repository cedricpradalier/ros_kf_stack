#ifndef TASK_FOLLOW_SHORE_PID_H
#define TASK_FOLLOW_SHORE_PID_H

#include "task_manager_lib/TaskDefinition.h"
#include "sim_tasks/SimTasksEnv.h"
#include "sim_tasks/TaskFollowShorePIDConfig.h"

using namespace task_manager_lib;

namespace sim_tasks {
    class TaskFollowShorePID : public TaskDefinitionWithConfig<TaskFollowShorePIDConfig, TaskFollowShorePID>
    {

        protected:
            boost::shared_ptr<SimTasksEnv> env;
            bool outOfStartBox;
            bool backToStartBox;
            ros::Publisher status_dist_pub;
	    ros::Publisher status_angle_pub;
 	    float angle_error_prev;
   	    float d_angle_error;
    	    float distance_error_prev;
 	    float d_distance_error;
 	    float i_angle_error;
  	    float i_distance_error;
        public:
            TaskFollowShorePID(boost::shared_ptr<TaskEnvironment> env); 
            virtual ~TaskFollowShorePID() {};

            virtual TaskIndicator initialise(const TaskParameters & parameters) throw (InvalidParameter);

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();
    };
};

#endif // TASK_FOLLOW_SHORE_PID_H
