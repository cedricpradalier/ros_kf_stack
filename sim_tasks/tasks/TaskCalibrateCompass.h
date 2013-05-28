#ifndef TASK_CALIBRATE_COMPASS_H
#define TASK_CALIBRATE_COMPASS_H

#include <ros/ros.h>
#include "task_manager_lib/TaskDefinition.h"
#include "sim_tasks/SimTasksEnv.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "sim_tasks/TaskCalibrateCompassConfig.h"

using namespace task_manager_lib;

namespace sim_tasks {
    class TaskCalibrateCompass : public TaskDefinitionWithConfig<TaskCalibrateCompassConfig, TaskCalibrateCompass>
    {

        protected:
            boost::shared_ptr<SimTasksEnv> env;
            ros::ServiceClient magOffsetClient;
            ros::Subscriber magSub;
            std::vector<geometry_msgs::Vector3> readings;
            double initial_heading;
            void magCallback(const geometry_msgs::Vector3StampedConstPtr& msg);
            // Small state machine to control the rotation
            enum {FIRST_HALF, SECOND_HALF} state;
        public:
            TaskCalibrateCompass(boost::shared_ptr<TaskEnvironment> env); 
            virtual ~TaskCalibrateCompass() {};

            virtual TaskIndicator initialise(const TaskParameters & parameters) throw (InvalidParameter);

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();
    };
};

#endif // TASK_CALIBRATE_COMPASS_H
