#ifndef TASK_CALIBRATE_COMPASS_H
#define TASK_CALIBRATE_COMPASS_H

#include <ros/ros.h>
#include "task_manager_lib/TaskDefinition.h"
#include "sim_tasks/SimTasksEnv.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "sim_tasks_cfg/TaskCalibrateCompassConfig.h"

using namespace task_manager_lib;
using namespace sim_tasks_cfg;

namespace sim_tasks {
    class TaskCalibrateCompass : public TaskInstance<TaskCalibrateCompassConfig,SimTasksEnv>
    {

        protected:
            ros::ServiceClient magOffsetClient;
            ros::Subscriber magSub;
            std::vector<geometry_msgs::Vector3> readings;
            double initial_heading;
            void magCallback(const geometry_msgs::Vector3StampedConstPtr& msg);
            // Small state machine to control the rotation
            enum {FIRST_HALF, SECOND_HALF} state;
        public:
            TaskCalibrateCompass(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskCalibrateCompass() {};

            virtual TaskIndicator initialise(const TaskParameters & parameters) ;

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();
    };
    class TaskFactoryCalibrateCompass : public TaskDefinition<TaskCalibrateCompassConfig, SimTasksEnv, TaskCalibrateCompass>
    {

        public:
            TaskFactoryCalibrateCompass(TaskEnvironmentPtr env) : 
                Parent("CalibrateCompass","Perform a rotation on the spot and update the compass calibration",true,env) {}
            virtual ~TaskFactoryCalibrateCompass() {};
    };
};

#endif // TASK_CALIBRATE_COMPASS_H
