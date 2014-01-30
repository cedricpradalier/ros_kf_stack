#include <math.h>
#include "TaskFollowShoreRP.h"
#include "sim_tasks_cfg/TaskFollowShoreConfig.h"
#include <nav_msgs/Path.h>




using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace sim_tasks_cfg;
using namespace sim_tasks;
using namespace radial_plan;

bool TaskFollowShoreRP::dump_images(std_srvs::Empty::Request  &req,
        std_srvs::Empty::Response &res)
{
    if (LP) {
        ROS_INFO("Dumping cell maps");
        LP->saveCellMaps();
    }
    return true;
}

// #define DEBUG_GOTO

TaskIndicator TaskFollowShoreRP::initialise(const TaskParameters & parameters) 
{
    path_pub = env->getNodeHandle().advertise<nav_msgs::Path>("follow_shore/path",1);
    if (cfg.radial) {
        RP.reset(new RadialPlan(cfg.radial_steps,cfg.angular_steps,cfg.connection_steps,
                    cfg.filter_glare,cfg.radial_resolution,cfg.angular_range));
    } else {
        LP.reset(new LocalPlan((cfg.side>0)?LocalPlan::LEFT:LocalPlan::RIGHT, cfg.distance, cfg.safety_distance,
                    cfg.forward_range,cfg.backward_range,cfg.radial_resolution,cfg.angular_steps,cfg.filter_glare));
        dump_srv = env->getNodeHandle().advertiseService("dump",&TaskFollowShoreRP::dump_images,this);
    }
    return TaskStatus::TASK_INITIALISED;
}

TaskIndicator TaskFollowShoreRP::iterate()
{

    const pcl::PointCloud<pcl::PointXYZ> & pointCloud = env->getPointCloud();

    // TODO: add parameters for these values
    std::list<cv::Point3f> lpath;
    if (RP) {
        RP->updateNodeCosts(pointCloud, (cfg.side>0)?RadialPlan::LEFT:RadialPlan::RIGHT, cfg.distance, cfg.safety_distance);
        lpath = RP->getOptimalPath(cfg.k_initial_angle, cfg.k_length, cfg.k_turn, cfg.k_dist);
    } else if (LP) {
        LP->updateCellCosts(pointCloud);
        lpath = LP->getOptimalPath(cfg.k_initial_angle, cfg.k_length, cfg.k_turn, cfg.k_dist);
    }

    // Finally create a ROS path message
    nav_msgs::Path path;
    path.header = env->getPointCloudHeader();
    path.poses.resize(lpath.size());
    std::list<cv::Point3f>::const_iterator it = lpath.begin();
    size_t ipose = 0;
    while (it != lpath.end()) {
        path.poses[ipose].header = path.header;
        path.poses[ipose].pose.position.x = it->x;
        path.poses[ipose].pose.position.y = it->y;
        tf::Quaternion Q = tf::createQuaternionFromRPY(0,0,it->z);
        tf::quaternionTFToMsg(Q,path.poses[ipose].pose.orientation);
        it ++;
        ipose ++;
    }
    path_pub.publish(path);

    if (lpath.empty()) {
        env->publishVelocity(0.0, 0.0);
    } else {
        // remove the first element, this is the current position
        lpath.pop_front();
        std::list<cv::Point3f>::const_iterator it = lpath.begin();
        // minus sign because the laser is upside down. 
        // Should probably be done through TF
        double alpha = 0;
        double sum_discount = 0.0;
        double discount = 1.0;
        while (it != lpath.end()) {
            alpha += -atan2(it->y,it->x) * discount;
            sum_discount += discount;
            discount *= cfg.alpha_discount;
            it ++;
        }
        alpha /= sum_discount;

        int salpha = (alpha<0)?-1:1;
        double rot = cfg.k_alpha * salpha * pow(fabs(alpha),cfg.alpha_power);
        rot = std::max(-cfg.max_ang_vel,std::min(cfg.max_ang_vel,rot));
        double rot_scale = rot / cfg.velocity_scaling;
        double vel = cfg.velocity * exp(-rot_scale*rot_scale);
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
