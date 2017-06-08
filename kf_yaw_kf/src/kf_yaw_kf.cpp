
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <Eigen/Core>
#include <Eigen/LU>

#include "kf_yaw_kf/CompassKF.h"
#include "kf_yaw_kf/SetMagOffset.h"

using namespace message_filters;
using namespace kf_yaw_kf;


class KFYawKF {
    protected:
        // State is yaw(i), yaw(i-1), omega, omega_bias
        Eigen::Vector3f X;
        Eigen::Matrix3f P, Q;
        Eigen::Matrix2f R;
        Eigen::Matrix3f eye3, A;
        Eigen::MatrixXf H,K;
        Eigen::Vector2f Z,I;

        // Data for mag_offset estimation
        double scale;
        double xcmin, ycmin;
        double mag_xmin, mag_xmax, mag_ymin, mag_ymax;
        Eigen::MatrixXf circle;
        Eigen::MatrixXf mag_accu;

        bool first, replay, debug_pub;
        Eigen::Vector3f mag_offset;

        kf_yaw_kf::CompassKF compass;

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
            X.setZero(3);
            P = eye3;
            first = true;
            ROS_INFO("Updated Magnetometer offset to %.2f %.2f %.2f",mag_offset(0),mag_offset(1),mag_offset(2));
            return true;
        }

        void kf_update(double phi_mag, double omega) {
            Z << phi_mag, omega;
            I = Z - H * X;
            I(0) = remainder(I(0),2*M_PI);
            Eigen::MatrixXf PHt = P * H.transpose();
            K = PHt * (H * PHt + R).inverse();
            X = X + K * I;
            P = (eye3  - K * H) * P;
        }

        void kf_predict(double dt) {
            A(0,1) = dt;
            X = A * X;
            X(0) = remainder(X(0),2*M_PI);
            P = A * P * A.transpose() + Q;
        }


        void update_mag_offset(const geometry_msgs::Vector3StampedConstPtr & mag) {
            double x = mag->vector.x; double y = mag->vector.y;
            int xo=round((mag_xmin - x - xcmin)/scale);
            int yo=round((mag_ymin - y - ycmin)/scale);
            mag_accu = 0.9999*mag_accu + circle.block(yo,xo,mag_accu.rows(),mag_accu.cols());
            Eigen::MatrixXf::Index u=0,v=0;
            double val = mag_accu.maxCoeff(&v,&u);
            mag_offset(0) = mag_xmin + u*scale;
            mag_offset(1) = mag_ymin + v*scale;
            printf("Offset: %f %f\n",mag_offset(0),mag_offset(1));
        }


        void imu_callback(const geometry_msgs::Vector3StampedConstPtr & mag,
                const geometry_msgs::Vector3StampedConstPtr & rpy,
                const sensor_msgs::ImuConstPtr & imu) {
            update_mag_offset(mag);
            std_msgs::Float32 magMsg;
            ros::Time now = mag->header.stamp;
            double omega = imu->angular_velocity.z;
            double yaw_mag = atan2(-(mag->vector.x - mag_offset(0)),-(mag->vector.y - mag_offset(1)));
            magMsg.data = yaw_mag;
            magPub.publish(magMsg);
            if (!first) {
                double dt = (now - last_stamp).toSec();
                kf_predict(dt);
                kf_update(M_PI/2-yaw_mag,omega);

                compass.heading = X(0) + heading_offset;
                compass.compass = remainder(M_PI/2. - compass.heading,2*M_PI);
                compass.heading_rate = X(1);
                compass.gyro_bias = X(2);
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
                ROS_INFO("Initialised compass filter to magnetometer value: %.2f",yaw_mag);
                X(0) = M_PI/2-yaw_mag;
                // X(1) = 1e-3;
            }
            first = false;
            last_stamp = now;
            last_yaw_gyro = rpy->vector.z;
        }


    public:
        KFYawKF() : H(2,3), K(3,2), nh("~") {
            eye3.setIdentity();
            A.setIdentity();
            X.setZero(3);
            P = eye3;
            first = true;
            debug_pub = false;
            replay = true;
            compass.header.frame_id = "/kingfisher/base";
            nh.getParam("frame_id",compass.header.frame_id);
            nh.getParam("debug_publishers",debug_pub);
            nh.getParam("replay",replay);
            double stddev_yaw_mag = 0.1; nh.getParam("stddev_yaw_mag",stddev_yaw_mag); 
            double stddev_yaw_gyro = 0.01; nh.getParam("stddev_yaw_gyro",stddev_yaw_gyro); 
            double stddev_omega = 0.01; nh.getParam("stddev_omega",stddev_omega); 
            heading_offset = 0.0; nh.getParam("heading_offset",heading_offset); 
            double mag_x_offset = 240.0; nh.getParam("mag_offset_x",mag_x_offset); 
            double mag_y_offset = 15.0; nh.getParam("mag_offset_y",mag_y_offset); 
            double mag_z_offset = 0.0; nh.getParam("mag_offset_z",mag_z_offset); 
            mag_offset(0) = mag_x_offset;
            mag_offset(1) = mag_y_offset;
            mag_offset(2) = mag_z_offset;

            scale = 4;
            xcmin = ycmin = -500;
            circle.resize(round(-2*ycmin/scale)+1,round(-2*xcmin/scale)+1);
            mag_xmin = mag_x_offset - 100; mag_xmax = mag_x_offset + 100;
            mag_ymin = mag_y_offset - 100; mag_ymax = mag_y_offset + 100;
            mag_accu.resize(round(200/scale),round(200/scale));
            for (unsigned int j=0;j<circle.rows();j++) {
                for (unsigned int i=0;i<circle.cols();i++) {
                    double r = hypot(scale*i+xcmin,scale*j+ycmin);
                    double mu = (r-145.)/10.;
                    circle(j,i) = exp(-0.5*mu*mu);
                }
            }
            for (unsigned int j=0;j<mag_accu.rows();j++) {
                for (unsigned int i=0;i<mag_accu.cols();i++) {
                    double mu=hypot(mag_xmin+scale*i-mag_x_offset,mag_ymin+scale*j-mag_y_offset)/30;
                    mag_accu(j,i) = 100. * exp(-0.5*mu*mu);
                }
            }

            ROS_INFO("Updated Heading offset %.2f,  Magnetometer offset to %.2f %.2f %.2f",heading_offset,mag_offset(0),mag_offset(1),mag_offset(2));

            Q << 1e-3,0,0,
              0,1e-3,0,
              0,0,1e-8;
            R << stddev_yaw_mag,0,
              0,stddev_omega;
            R = R * R;

            H << 1,0,0,
              0,1,1;

            offsetService = nh.advertiseService("mag_offset",&KFYawKF::set_mag_offset,this);
            magPub = nh.advertise<std_msgs::Float32>("mag",1);
            if (replay) {
                // could be done through remap
                compPub = nh.advertise<kf_yaw_kf::CompassKF>("compass2",1);
            } else {
                compPub = nh.advertise<kf_yaw_kf::CompassKF>("compass",1);
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




