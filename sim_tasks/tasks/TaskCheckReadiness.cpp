
#include <math.h>
#include <sim_tasks/SubscriberStatistics.h>
#include "TaskCheckReadiness.h"
#include "sim_tasks_cfg/TaskCheckReadinessConfig.h"

using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace sim_tasks_cfg;
using namespace sim_tasks;

TaskIndicator TaskCheckReadiness::initialise(const TaskParameters & parameters) 
{
    initial_time = ros::Time::now();
    return TaskStatus::TASK_INITIALISED;
    SubscriberStatMap::iterator it;
    for (it=statMap.begin();it!=statMap.end();it++) {
        it->second.setMark();
    }
    for (it=env->getStatisticsMap().begin();it!=env->getStatisticsMap().end();it++) {
        it->second.setMark();
    }
    jpegSub = env->getNodeHandle().subscribe("/axis/image_raw/compressed",1,&TaskCheckReadiness::jpegCallback,this);
    logbeatSub = env->getNodeHandle().subscribe("/logbeat/beat",1,&TaskCheckReadiness::logbeatCallback,this);
    tfbeatSub = env->getNodeHandle().subscribe("/tfbeat/beat",1,&TaskCheckReadiness::tfbeatCallback,this);
}

void TaskCheckReadiness::jpegCallback(const sensor_msgs::CompressedImage::ConstPtr& msg) {
    // don't care about the value here
    statMap["jpeg"].tick();
}

void TaskCheckReadiness::logbeatCallback(const std_msgs::Header::ConstPtr& msg) {
    // don't care about the value here
    statMap["logbeat"].tick();
}

void TaskCheckReadiness::tfbeatCallback(const std_msgs::Header::ConstPtr& msg) {
    // don't care about the value here
    statMap["tfbeat"].tick();
}

#define L_TEST(field,min_freq,text) \
        if (statMap[field].getFrequency(true)<min_freq) { \
            ROS_ERROR("Not enough messages received for %s: %d msg in %.1f",text,statMap[field].getCount(true),cfg.duration); \
            setStatusString(std::string("Not enough messages received for ")+text); \
            return TaskStatus::TASK_FAILED; \
        }

#define E_TEST(field,min_freq,text) \
        if (env->getStatisticsMap()[field].getFrequency(true)<min_freq) { \
            ROS_ERROR("Not enough messages received for %s: %d msg in %.1f",text,env->getStatisticsMap()[field].getCount(true),cfg.duration); \
            setStatusString(std::string("Not enough messages received for ")+text); \
            return TaskStatus::TASK_FAILED; \
        }

TaskIndicator TaskCheckReadiness::iterate()
{
    if ((ros::Time::now() - initial_time).toSec() > cfg.duration) {
        L_TEST("jpeg",5.0,"AXIS Images");
        L_TEST("logbeat",1.0,"Log beats");
        E_TEST("utm",0.5,"UTM messages");
        E_TEST("compass",5.0,"Compass messages");
        E_TEST("sense",5.0,"Sense messages");
        E_TEST("axis",5.0,"Axis state messages");
        if ((env->getStatisticsMap()["pointcloud"].getFrequency(true)<5.0)
                && (env->getStatisticsMap()["scan"].getFrequency(true)<10.0)) { 
            ROS_ERROR("Not enough messages received for laser point cloud"); 
            return TaskStatus::TASK_FAILED; 
        }
        return TaskStatus::TASK_COMPLETED;
    }
	return TaskStatus::TASK_RUNNING;
}

TaskIndicator TaskCheckReadiness::terminate()
{
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskFactoryCheckReadiness);
