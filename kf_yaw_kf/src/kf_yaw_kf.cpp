
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <Eigen/Core>
#include <Eigen/LU>

#include "kf_yaw_kf/Compass.h"
#include "kf_yaw_kf/SetMagOffset.h"

using namespace message_filters;
using namespace kf_yaw_kf;


class KFYawKF {
    protected:
        // State is yaw(i), yaw(i-1), omega, omega_bias
        Eigen::Vector4f X;
        Eigen::Matrix4f P, Q;
        Eigen::Matrix3f R;
        Eigen::Matrix4f eye4, A;
        Eigen::MatrixXf H,K;
        Eigen::Vector3f Z,I;

        bool first, replay, debug_pub;
        Eigen::Vector3f mag_offset;

        kf_yaw_kf::Compass compass;

        ros::Time last_stamp;
        double last_yaw_gyro;
        double heading_offset;

        ros::NodeHandle nh;
        ros::ServiceServer offsetService;
        ros::Publisher compPub, magPub, qPub, pPub;
        boost::shared_ptr< message_filters::Subscriber<geometry_msgs::Vector3Stamped> > magSub;
        boost::shared_ptr< message_filters::Subscriber<geometry_msgs::Vector3Stamped> > rpySub;
        boost::shared_ptr< message_filters::Subscriber<sensor_msgs::Imu> > imuSub;
        typedef TimeSynchronizer<geometry_msgs::Vector3Stamped, geometry_msgs::Vector3Stamped, sensor_msgs::Imu> TS;
        boost::shared_ptr<TS> sync;

        bool set_mag_offset(SetMagOffset::Request & req, SetMagOffset::Response & res) {
            mag_offset(0) = req.mag_x_offset;
            mag_offset(1) = req.mag_y_offset;
            mag_offset(2) = req.mag_z_offset;
            X.setZero(4);
            P = eye4;
            first = true;
            ROS_INFO("Updated Magnetometer offset to %.2f %.2f %.2f",mag_offset(0),mag_offset(1),mag_offset(2));
            return true;
        }

        void kf_update(double dt, double phi_mag, double dphi_gyro, double omega) {
            H(1,3) = dt;
            Z << phi_mag, dphi_gyro, omega;
            I = Z - H * X;
            I(0) = remainder(I(0),2*M_PI);
            I(1) = remainder(I(1),2*M_PI);
            Eigen::MatrixXf PHt = P * H.transpose();
            K = PHt * (H * PHt + R).inverse();
            X = X + K * I;
            P = (eye4  - K * H) * P;
        }

        void kf_predict(double dt) {
            A(0,2) = dt;
            X = A * X;
            X(0) = remainder(X(0),2*M_PI);
            P = A * P * A.transpose() + Q;
        }

        void imu_callback(const geometry_msgs::Vector3StampedConstPtr & mag,
                const geometry_msgs::Vector3StampedConstPtr & rpy,
                const sensor_msgs::ImuConstPtr & imu) {
            std_msgs::Float32 magMsg;
            ros::Time now = mag->header.stamp;
            double omega = - imu->angular_velocity.z;
            double yaw_mag = atan2(-(mag->vector.x - mag_offset(0)),-(mag->vector.y - mag_offset(1)));
            magMsg.data = yaw_mag;
            magPub.publish(magMsg);
            if (!first) {
                double dt = (now - last_stamp).toSec();
                double dphi_gyro = -(rpy->vector.z - last_yaw_gyro);
                kf_predict(dt);
                kf_update(dt,yaw_mag,dphi_gyro,omega);

                compass.heading = X(0) + heading_offset;
                compass.compass = remainder(M_PI/2. - compass.heading,2*M_PI);
                compass.stddev = sqrt(P(0,0));
                if (replay) {
                    compass.header.stamp = ros::Time::now();
                } else {
                    compass.header.stamp = now;
                }
                compPub.publish(compass);
                if (debug_pub) {
                    geometry_msgs::Vector3 q;
                    q.x = X(0); q.y = X(1); q.z = X(2);
                    geometry_msgs::Vector3 p;
                    p.x = P(0,0); p.y = P(1,1); p.z = P(2,2);
                    qPub.publish(q);
                    pPub.publish(p);
                }
            } else {
                X(0) = X(1) = yaw_mag;
            }
            first = false;
            last_stamp = now;
            last_yaw_gyro = rpy->vector.z;
        }


    public:
        KFYawKF() : H(3,4), K(4,3), nh("~") {
            eye4.setIdentity();
            A.setIdentity();
            A(1,0) = 1; A(1,1) = 0;
            X.setZero(4);
            P = eye4;
            first = true;
            debug_pub = false;
            compass.header.frame_id = "/kingfisher/base";
            nh.getParam("frame_id",compass.header.frame_id);
            nh.getParam("debug_publishers",debug_pub);
            nh.getParam("replay",replay);
            double stddev_yaw_mag = 0.1; nh.getParam("stddev_yaw_mag",stddev_yaw_mag); 
            double stddev_yaw_gyro = 0.01; nh.getParam("stddev_yaw_gyro",stddev_yaw_gyro); 
            double stddev_omega = 0.01; nh.getParam("stddev_omega",stddev_omega); 
            heading_offset = 0.0; nh.getParam("heading_offset",heading_offset); 
            double mag_x_offset = 0.0; nh.getParam("mag_offset_x",mag_x_offset); 
            double mag_y_offset = 0.0; nh.getParam("mag_offset_y",mag_y_offset); 
            double mag_z_offset = 0.0; nh.getParam("mag_offset_z",mag_z_offset); 
            mag_offset(0) = mag_x_offset;
            mag_offset(1) = mag_y_offset;
            mag_offset(2) = mag_z_offset;
            ROS_INFO("Updated Heading offset %.2f,  Magnetometer offset to %.2f %.2f %.2f",heading_offset,mag_offset(0),mag_offset(1),mag_offset(2));
            Q << 1e-3,0,0,0,
              0,1e-5,0,0,
              0,0,1e-3,0,
              0,0,0,1e-5;
            R << stddev_yaw_mag,0,0,
              0,stddev_yaw_gyro,0,
              0,0,stddev_omega;
            R = R * R;

            H << 1,0,0,0,
              1,-1,0,0,
              0,0,1,0;

            offsetService = nh.advertiseService("mag_offset",&KFYawKF::set_mag_offset,this);
            magPub = nh.advertise<std_msgs::Float32>("mag",1);
            if (replay) {
                // could be done through remap
                compPub = nh.advertise<kf_yaw_kf::Compass>("compass2",1);
            } else {
                compPub = nh.advertise<kf_yaw_kf::Compass>("compass",1);
            }
            if (debug_pub) {
                qPub = nh.advertise<geometry_msgs::Vector3>("state",1);
                pPub = nh.advertise<geometry_msgs::Vector3>("cov",1);
            }
            
            imuSub.reset(new message_filters::Subscriber<sensor_msgs::Imu>(nh,"/imu/data",1));
            magSub.reset(new message_filters::Subscriber<geometry_msgs::Vector3Stamped>(nh,"/imu/mag",1));
            rpySub.reset(new message_filters::Subscriber<geometry_msgs::Vector3Stamped>(nh,"/imu/rpy",1));
            sync.reset(new TS(*magSub,*rpySub,*imuSub,50));
            sync->registerCallback(boost::bind(&KFYawKF::imu_callback, this, _1, _2, _3));

            ROS_INFO("Compass up and running");
        }
};

int main(int argc, char * argv[])
{
    ros::init(argc,argv,"compass");
    KFYawKF filter;

    ros::spin();
    return 0;
}




