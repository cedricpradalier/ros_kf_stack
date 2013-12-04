
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
    return TaskStatus::TASK_INITIALISED;
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
            ROS_ERROR("Not enough messages received for %s: %d msg / %.2f Hz in %.1fs",text,(int)statMap[field].getCount(true),statMap[field].getFrequency(true),cfg.duration); \
            setStatusString(std::string("Not enough messages received for ")+text); \
            readiness_ok = false; \
        } else { \
            ROS_INFO("Received enough messages for %s: %d msg / %.2f Hz in %.1fs",text,(int)statMap[field].getCount(true),statMap[field].getFrequency(true),cfg.duration); \
        }

#define E_TEST(field,min_freq,text) \
        if (env->getStatisticsMap()[field].getFrequency(true)<min_freq) { \
            ROS_ERROR("Not enough messages received for %s: %d msg / %.2f Hz in %.1fs",text,(int)env->getStatisticsMap()[field].getCount(true),env->getStatisticsMap()[field].getFrequency(true),cfg.duration); \
            setStatusString(std::string("Not enough messages received for ")+text); \
            readiness_ok = false; \
        } else { \
            ROS_INFO("Received enough messages for %s: %d msg / %.2f Hz in %.1fs",text,(int)env->getStatisticsMap()[field].getCount(true),env->getStatisticsMap()[field].getFrequency(true),cfg.duration); \
        }

TaskIndicator TaskCheckReadiness::iterate()
{
    if ((ros::Time::now() - initial_time).toSec() > cfg.duration) {
        bool readiness_ok = true;
        if (cfg.logger) {
            L_TEST("logbeat",1.0,"Log beats");
        }
        if (cfg.subscribers) {
            L_TEST("jpeg",5.0,"AXIS Images");
            L_TEST("tfbeat",1.0,"Log beats");
            E_TEST("utm",0.5,"UTM messages");
            E_TEST("compass",5.0,"Compass messages");
            E_TEST("sense",5.0,"Sense messages");
            E_TEST("axis",5.0,"Axis state messages");
            if ((env->getStatisticsMap()["pointcloud"].getFrequency(true)<5.0)
                    && (env->getStatisticsMap()["scan"].getFrequency(true)<10.0)) { 
                ROS_ERROR("Not enough messages received for laser point cloud"); 
                readiness_ok = false;
            } else {
                ROS_INFO("Received enough messages for proximity: ls %d msg / %.2f Hz pc %d msg / %.2f Hz in %.1fs",
                        (int)env->getStatisticsMap()["scan"].getCount(true),
                        env->getStatisticsMap()["scan"].getFrequency(true),
                        (int)env->getStatisticsMap()["pointcloud"].getCount(true),
                        env->getStatisticsMap()["pointcloud"].getFrequency(true),
                        cfg.duration); 
            }
        }
        if (!readiness_ok) {
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
