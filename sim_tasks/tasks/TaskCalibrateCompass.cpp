
#include <math.h>
#include "TaskCalibrateCompass.h"
#include "kf_yaw_kf/SetMagOffset.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "sim_tasks/TaskCalibrateCompassConfig.h"

#include <Eigen/Core>

using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace sim_tasks;

// #define DEBUG_GOTO

TaskIndicator TaskCalibrateCompass::initialise(const TaskParameters & parameters) throw (InvalidParameter)
{
    TaskIndicator ti = Parent::initialise(parameters);
    if (ti != TaskStatus::TASK_INITIALISED) {
        return ti;
    }
    // Now we can do any required task-specific initialisation, for instance
    // creating the service proxy and subscriber. They will be destroyed when
    // the task terminates
    initial_heading = env->getHeading();
    state = FIRST_HALF;
    readings.clear();
    magOffsetClient = env->getNodeHandle().serviceClient<kf_yaw_kf::SetMagOffset>("/compass/mag_offset");
    magSub = env->getNodeHandle().subscribe("/imu/mag",1,&TaskCalibrateCompass::magCallback,this);
    ROS_INFO("Starting magnetometer calibration rotation");
    return TaskStatus::TASK_INITIALISED;
}

void TaskCalibrateCompass::magCallback(const geometry_msgs::Vector3StampedConstPtr& msg) {
    readings.push_back(msg->vector);
}



TaskIndicator TaskCalibrateCompass::iterate()
{
    double heading = env->getHeading();
    double alpha = remainder(heading-initial_heading,2*M_PI);
    switch (state) {
        case FIRST_HALF:
            if (fabs(alpha)>M_PI/2) {
                state = SECOND_HALF;
                ROS_INFO("Magnetometer calibration: waiting for rotation completion");
            }
            break;
        case SECOND_HALF:
            if (fabs(alpha)<cfg.angle_threshold) {
                if (readings.size() < 20) {
                    ROS_ERROR("Did not receive enough magnetometer data while calibrating compass (%d).",(int)readings.size());
                    return TaskStatus::TASK_FAILED;
                }
                return TaskStatus::TASK_COMPLETED;
            }
            break;
    };
    env->publishVelocity(0.0, cfg.angular_velocity);
	return TaskStatus::TASK_RUNNING;
}

TaskIndicator TaskCalibrateCompass::terminate()
{
    env->publishVelocity(0,0);
    if (readings.size() > 20) {
        ROS_INFO("Completed calibration rotation. Computing offset");
        // Now compute the circle center and udpate the estimator
        //  http://www.dtcenter.org/met/users/docs/write_ups/circle_fit.pdf
        double m_x=0, m_y=0, m_z=0;
        for (unsigned int i=0;i<readings.size();i++) {
            m_x += readings[i].x;
            m_y += readings[i].y;
            m_z += readings[i].z;
        }
        m_x /= readings.size();
        m_y /= readings.size();
        m_z /= readings.size();
        double Suu=0, Svv=0, Suv=0;
        double Suuu=0, Suvv=0, Svuu=0, Svvv=0;
        for (unsigned int i=0;i<readings.size();i++) {
            double u = readings[i].x - m_x;
            double v = readings[i].y - m_y;
            double uu = u*u;
            double uv = u*v;
            double vv = v*v;
            double uuu = u * uu;
            double uvv = u * vv;
            double vuu = v * uu;
            double vvv = v * vv;
            Suu += uu; Suv += uv; Svv += vv;
            Suuu += uuu; Suvv += uvv; Svuu += vuu; Svvv += vvv;
        };
        Eigen::Vector2f B; 
        B << Suuu+Suvv, Svvv+Svuu;
        Eigen::Matrix2f A; 
        A << Suu, Suv,  
          Suv, Svv;
        Eigen::Vector2f U = 0.5 * A.inverse() * B; // U contains the circle center (minus the mean)
        ROS_INFO("Magnetometer calibration offset is %.2f %.2f",U(0)+m_x,U(1)+m_y);

        // No need to compute the radius in this application, otherwise
        // double R = sqrt(U(0)*U(0) + U(1)*U(1) + (Suu + Svv)/reading.size());
        
        kf_yaw_kf::SetMagOffset smo;
        smo.request.mag_x_offset = U(0)+m_x;
        smo.request.mag_y_offset = U(1)+m_y;
        smo.request.mag_z_offset = m_z;
        if (!magOffsetClient.call(smo)) {
            ROS_ERROR("CalibrateCompass: Failed to call service /compass/mag_offset");
        }
    }
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskFactoryCalibrateCompass);
