#include <math.h>
#include "TaskFollowShoreRP.h"
#include "sim_tasks_cfg/TaskFollowShoreConfig.h"
#include <nav_msgs/Path.h>
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace sim_tasks_cfg;
using namespace sim_tasks;
using namespace radial_plan;

// #define DEBUG_GOTO

TaskIndicator TaskFollowShoreRP::initialise(const TaskParameters & parameters) 
{
    path_pub = env->getNodeHandle().advertise<nav_msgs::Path>("follow_shore/path",1);
    // TODO: add parameters for these values
    RP.reset(new RadialPlan(cfg.radial_steps,cfg.angular_steps,cfg.connection_steps,
                cfg.radial_resolution,cfg.angular_range));
    return TaskStatus::TASK_INITIALISED;
}

TaskIndicator TaskFollowShoreRP::iterate()
{
    boost::lock_guard<boost::mutex> guard(env->getMutex());

    const pcl::PointCloud<pcl::PointXYZ> & pointCloud = env->getPointCloud();

    // TODO: add parameters for these values
    RP->updateNodeCosts(pointCloud, (cfg.side>0)?RadialPlan::LEFT:RadialPlan::RIGHT, cfg.distance, cfg.safety_distance);
    std::list<cv::Point2f> lpath;
    lpath = RP->getOptimalPath(cfg.k_initial_angle, cfg.k_length, cfg.k_turn, cfg.k_dist);

    // Finally create a ROS path message
    nav_msgs::Path path;
    path.header = env->getPointCloudHeader();
    path.poses.resize(lpath.size());
    std::list<cv::Point2f>::const_iterator it = lpath.begin();
    unsigned int ipose = 0;
    while (it != lpath.end()) {
        // time stamp is not updated because we're not creating a
        // trajectory at this stage
        path.poses[ipose].header = path.header;
        path.poses[ipose].pose.position.x = it->x;
        path.poses[ipose].pose.position.y = it->y;
        if (ipose > 0) {
            const geometry_msgs::Point & prev = path.poses[ipose-1].pose.position;
            tf::Quaternion Q = tf::createQuaternionFromRPY(0,0,atan2(it->y-prev.y,it->x-prev.x));
            tf::quaternionTFToMsg(Q,path.poses[ipose].pose.orientation);
        } else {
            tf::Quaternion Q = tf::createQuaternionFromRPY(0,0,0);
            tf::quaternionTFToMsg(Q,path.poses[ipose].pose.orientation);
        }
        ipose++;
        it ++;
    }
    path_pub.publish(path);

    if (lpath.empty()) {
        env->publishVelocity(0.0, 0.0);
    } else {
        lpath.pop_front();
        cv::Point2f P = lpath.front();
        // minus sign because the laser is upside down. 
        // Should probably be done through TF
        float alpha = -atan2(P.y,P.x);
        float rot = cfg.k_alpha * alpha;
        float rot_scale = rot / cfg.velocity_scaling;
        float vel = cfg.velocity * exp(-rot_scale*rot_scale);
        env->publishVelocity(vel, rot);
    }

	return TaskStatus::TASK_RUNNING;
}

TaskIndicator TaskFollowShoreRP::terminate()
{
    env->publishVelocity(0,0);
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskFactoryFollowShoreRP);
