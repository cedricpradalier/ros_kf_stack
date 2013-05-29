#include <math.h>
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

TaskIndicator TaskFollowShorePID::iterate()
{

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

    double vel = cfg.velocity;
    double rot = 0;

    float mindistance=1000;
    float theta_closest=0;
    float distance_i=0;
    float theta_i=0;
    float angle_error_prev;
    float d_angle_error;
    float distance_error_prev;
    float d_distance_error;
    float i_angle_error;
    float i_distance_error;
    float i_max = 10.0;
    float d_max = 5.0;

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

#ifdef DEBUG_GOTO
    ROS_INFO("pointCloudSize %d - mindistance %.3f - theta_closest %.3f",(int)pointCloud.size(),mindistance, theta_closest);
#endif
    
    float angle_error=0;
    float distance_error=0;


    if (mindistance > cfg.dist_threshold) {
        //TODO test the oldness of the pointcloud
        ROS_INFO("No shore detected");
        return TaskStatus::TASK_RUNNING;
    } else {
	// Error Calculations - 6 calculations, position error, deriv of position error and int of position error
	// then the same for the angular error, deriv of angle error and int of angle error.
        angle_error = remainder(cfg.angle-theta_closest,2*M_PI);
        distance_error = cfg.distance-mindistance;

        d_angle_error = (angle_error-angle_error_prev)/cfg.task_period;
        d_distance_error = (distance_error-distance_error_prev)/cfg.task_period;
	// saturate D's
	if (fabs(d_angle_error) > d_max) {
            d_angle_error = ((d_angle_error>0)?+1:-1) * d_max;
        }
	if (fabs(d_distance_error) > d_max) {
            d_distance_error = ((d_distance_error>0)?+1:-1) * d_max;
        }

        i_angle_error += cfg.task_period * angle_error;
        i_distance_error += cfg.task_period * distance_error;
	// saturate I's
	if (fabs(i_angle_error) > i_max) {
            i_angle_error = ((i_angle_error>0)?+1:-1) * i_max;
        }
	if (fabs(i_distance_error) > i_max) {
            i_distance_error = ((i_distance_error>0)?+1:-1) * i_max;
        }
	// Rotation calculation
        rot = - cfg.p_alpha * angle_error - ((cfg.angle>0)?+1:-1) * cfg.p_d * distance_error - cfg.d_alpha * d_angle_error - cfg.d_d * d_distance_error - cfg.i_alpha * i_angle_error - cfg.i_d * i_distance_error;
	// Record errors as previous values for d calculation
	angle_error_prev = angle_error;
	distance_error_prev = distance_error;
	// Saturation
        if (fabs(rot) > cfg.max_ang_vel) {
            rot = ((rot>0)?+1:-1) * cfg.max_ang_vel;
        }
    }
#ifdef DEBUG_GOTO
    ROS_INFO("Command vel %.2f angle_error %.2f distance_error %.2f rot %.2f\n",vel,angle_error,distance_error,rot);
#endif

    env->publishVelocity(vel, rot);
	return TaskStatus::TASK_RUNNING;
}

TaskIndicator TaskFollowShorePID::terminate()
{
    env->publishVelocity(0,0);
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskFollowShorePID);