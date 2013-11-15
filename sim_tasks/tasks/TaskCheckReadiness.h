#ifndef TASK_CHECK_READINESS_H
#define TASK_CHECK_READINESS_H

#include <ros/ros.h>
#include "task_manager_lib/TaskDefinition.h"
#include "sim_tasks/SimTasksEnv.h"
#include "sim_tasks_cfg/TaskCheckReadinessConfig.h"
#include <sensor_msgs/CompressedImage.h>
#include <std_msgs/Header.h>

using namespace task_manager_lib;
using namespace sim_tasks_cfg;

namespace sim_tasks {
    class TaskCheckReadiness : public TaskInstance<TaskCheckReadinessConfig,SimTasksEnv>
    {

        protected:
            ros::Time initial_time;
            ros::Subscriber jpegSub;
            ros::Subscriber logbeatSub;
            SubscriberStatMap statMap;
            void jpegCallback(const sensor_msgs::CompressedImage::ConstPtr& msg);
            void logbeatCallback(const std_msgs::Header::ConstPtr& msg);
        public:
            TaskCheckReadiness(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {
                statMap["jpeg"] = SubscriberStatistics();
                statMap["logbeat"] = SubscriberStatistics();
            }
            virtual ~TaskCheckReadiness() {};

            virtual TaskIndicator initialise(const TaskParameters & parameters) ;

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();
    };
    class TaskFactoryCheckReadiness : public TaskDefinition<TaskCheckReadinessConfig, SimTasksEnv, TaskCheckReadiness>
    {

        public:
            TaskFactoryCheckReadiness(TaskEnvironmentPtr env) : 
                Parent("CheckReadiness","Check that we're ready to run a mission",true,env) {}
            virtual ~TaskFactoryCheckReadiness() {};
    };
};

#endif // TASK_CHECK_READINESS_H
