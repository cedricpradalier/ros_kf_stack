#include <math.h>
#include <tf/transform_broadcaster.h>
#include "TaskFollowShorePID.h"
#include "sim_tasks/TaskFollowShorePIDConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace sim_tasks;

// #define DEBUG_GOTO

TaskFollowShorePID::TaskFollowShorePID(boost::shared_ptr<TaskEnvironment> tenv) 
    : TaskDefinitionWithConfig<TaskFollowShorePIDConfig,TaskFollowShorePID>("FollowShorePID","Follow the shore of the lake with PID controller",true,-1.)
{
    env = boost::dynamic_pointer_cast<SimTasksEnv,TaskEnvironment>(tenv);
    outOfStartBox = false;
    backToStartBox = false;
}

TaskIndicator TaskFollowShorePID::initialise(const TaskParameters & parameters) throw (InvalidParameter)
{
    TaskIndicator ti = Parent::initialise(parameters);
    if (ti != TaskStatus::TASK_INITIALISED) {
        return ti;
    }
    angle_error_prev=0.0;
    distance_error_prev=0.0;
    i_angle_error=0.0;
    i_distance_error=0.0;
    status_dist_pub = env->getNodeHandle().advertise<geometry_msgs::Vector3>("follow_shore/dist_status",1);
    status_angle_pub = env->getNodeHandle().advertise<geometry_msgs::Vector3>("follow_shore/angle_status",1);
    return TaskStatus::TASK_INITIALISED;
}

TaskIndicator TaskFollowShorePID::iterate()
{
    geometry_msgs::Vector3 dist;
    geometry_msgs::Vector3 angle;
    const geometry_msgs::Pose2D & tpose = env->getPose2D();
    const geometry_msgs::Pose2D & finishLine = env->getFinishLine2D();
    double scalarProduct = cos(finishLine.theta)*(finishLine.x-tpose.x)+sin(finishLine.theta)*(finishLine.y-tpose.y);
    double r = hypot(finishLine.y-tpose.y,finishLine.x-tpose.x);

#ifdef DEBUG_GOTO
    ROS_INFO("scalarProduct %.3f - dist_goal %.3f\n",scalarProduct,r);
#endif


    if ((backToStartBox) && (fabs(scalarProduct) < 0.1)) {
        return TaskStatus::TASK_COMPLETED;
    } else if ((outOfStartBox) && (r < cfg.dist_goal)) {
        ROS_INFO("Back to starbucks");
        backToStartBox = true;
    } else if ((!outOfStartBox) && (r > cfg.dist_goal)) {
        ROS_INFO("Out of starbucks");
        outOfStartBox = true;
    }

    const pcl::PointCloud<pcl::PointXYZ> & pointCloud = env->getPointCloud();

    float mindistance=1000;
    float theta_closest=0;
    float distance_i=0;
    float theta_i=0;


    for (unsigned int i=0;i<pointCloud.size();i++) {
        theta_i=-atan2(pointCloud[i].y,pointCloud[i].x);
//        if ((fabs(remainder(cfg.angle-theta_i,2*M_PI))<M_PI/2)&&(fabs(theta_i)<cfg.angle_range)) {
            distance_i=hypot(pointCloud[i].y,pointCloud[i].x);
            if ((distance_i < mindistance)&&(distance_i > 0.01)) {
	        mindistance=distance_i;
                theta_closest=theta_i;
            }
//        }
    }
    tf::Transform transform;
    std_msgs::Header hdr = env->getPointCloudHeader();
    if (hdr.frame_id != "") {
        transform.setOrigin( tf::Vector3(mindistance*cos(-theta_closest), mindistance*sin(-theta_closest), 0.0) );
        transform.setRotation( tf::createQuaternionFromRPY(0, 0, -theta_closest) );
        br.sendTransform(tf::StampedTransform(transform, hdr.stamp, hdr.frame_id, "/kingfisher/closest_point"));
    } else {
        transform.setOrigin( tf::Vector3(mindistance*cos(-theta_closest), mindistance*sin(-theta_closest), 0.0) );
        transform.setRotation( tf::createQuaternionFromRPY(0, 0, -theta_closest) );
        br.sendTransform(tf::StampedTransform(transform, hdr.stamp, "/kingfisher/laser", "/kingfisher/closest_point"));
    }


#ifdef DEBUG_GOTO
    ROS_INFO("pointCloudSize %d - mindistance %.3f - theta_closest %.3f",(int)pointCloud.size(),mindistance, theta_closest);
#endif
    float vel=0.0, rot=0.0, angle_error, distance_error, rot_ang, rot_lin;
    
    if (mindistance > cfg.dist_threshold) {
        i_angle_error=0;
        i_distance_error=0;
        angle_error_prev=0;
        distance_error_prev=0;

        return TaskStatus::TASK_RUNNING;
    } else {
        // Error Calculations - 6 calculations, position error, deriv of position error and int of position error
        // then the same for the angular error, deriv of angle error and int of angle error.
        // P error
        angle_error = remainder(cfg.angle-theta_closest,2*M_PI);
        distance_error = saturate(cfg.distance-mindistance,cfg.max_dist_error);

        // D Error
        d_angle_error = (angle_error-angle_error_prev)/cfg.task_period;
        d_distance_error = (distance_error-distance_error_prev)/cfg.task_period;

        // I error
        i_angle_error += cfg.task_period * angle_error;
        i_angle_error = saturate(i_angle_error,cfg.i_alpha_max);
        i_distance_error += cfg.task_period * distance_error;
        i_distance_error = saturate(i_distance_error,cfg.i_d_max);

        // Publish PID errors
        dist.x=distance_error;
        dist.y=d_distance_error;
        dist.z=i_distance_error;
        angle.x=angle_error;
        angle.y=d_angle_error;
        angle.z=i_angle_error;

        // Calculate angular component
        rot_ang = - cfg.p_alpha * angle_error - cfg.d_alpha * d_angle_error - cfg.i_alpha * i_angle_error;
        // Calculate linear component
        rot_lin = - ((cfg.angle>0)?+1:-1) * cfg.p_d * distance_error - cfg.d_d * d_distance_error - cfg.i_d * i_distance_error;

        // Saturate components
        // if (fabs(rot_ang) > E_ang_max) {
        //     rot_ang = ((rot_ang>0)?+1:-1) * E_ang_max;
        // }
        // if (fabs(rot_lin) > E_lin_max) {
        //     rot_lin = ((rot_lin>0)?+1:-1) * E_lin_max;
        // }	

        // Rotation calculation
        rot = rot_ang + rot_lin*exp(-0.5*angle_error*angle_error);

        // Record errors as previous values for d calculation
        angle_error_prev = angle_error;
        distance_error_prev = distance_error;
        // Saturation and Curvature Limitation
        float wnorm =std::max(0.0,fabs(rot)-cfg.rot_tol); 
        vel = exp(-cfg.K_E*(wnorm*wnorm)) * cfg.max_lin_vel;
        rot = saturate(rot,cfg.max_ang_vel);
    }
#ifdef DEBUG_GOTO
    ROS_INFO("Command vel %.2f angle_error %.2f distance_error %.2f rot %.2f\n",vel,angle_error,distance_error,rot);
#endif

    status_dist_pub.publish(dist);
    status_angle_pub.publish(angle);
    env->publishVelocity(vel, rot);
	return TaskStatus::TASK_RUNNING;
}

TaskIndicator TaskFollowShorePID::terminate()
{
    env->publishVelocity(0,0);
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskFollowShorePID);
