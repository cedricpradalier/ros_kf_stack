#include <stdlib.h>
#include <stdio.h>

#include <vector>
#include <string>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <axis_camera/Axis.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <image_transport/transport_hints.h>





class AxisExposureControl {
    public:
        typedef enum {
            FIXED_WINDOW,
            LASER_LINE
        } ControlType;

    protected:
        ros::NodeHandle nh_;
        image_transport::Subscriber tsub_;
        image_transport::ImageTransport it_;
        ros::Publisher axis_pub_;
        tf::TransformListener listener_;

        ControlType ctrl_type;
        double gain;
        int sampling;
        int first_row, last_row;
        int exposure;

    public:
        AxisExposureControl() : nh_("~"), it_(nh_) {
            std::string ctrl = "default";
            std::string transport = "raw";
            nh_.param("transport",transport,transport);
            nh_.param("sampling",sampling,10);
            nh_.param("first_row",first_row,0);
            nh_.param("last_row",last_row,0);
            nh_.param("gain",gain,10.0);
            nh_.param("initial_exposure",exposure,5000);
            nh_.param("control_type",ctrl,ctrl);
            if (ctrl == "laser") {
                ctrl_type = LASER_LINE;
            } else if ((ctrl == "fixed") || (ctrl == "default")) {
                ctrl_type = FIXED_WINDOW;
            }
            tsub_ = it_.subscribe<AxisExposureControl>("image",1, &AxisExposureControl::callback,this,transport);
            axis_pub_ = nh_.advertise<axis_camera::Axis>("axis",1);
        }

        void callback(const sensor_msgs::ImageConstPtr& msg) {
            cv::Mat_<uint8_t> view;
            size_t first = first_row;
            size_t last = (last_row?(std::max(first_row, last_row)):msg->width);
            cv::Rect rect(0,first,msg->height,last-first);
            if (msg->encoding == "mono8") {
                view = (cv_bridge::toCvShare(msg,"mono8")->image)(rect);
            } else if (msg->encoding == "rgb8") {
                cvtColor((cv_bridge::toCvShare(msg,"rgb8")->image)(rect),view,CV_RGB2GRAY);
            } else {
                ROS_ERROR("Unknown encoding '%s'",msg->encoding.c_str());
                return;
            }
            
            switch (ctrl_type) {
                case FIXED_WINDOW:
                    { 
                        size_t count = 0;
                        float avg = 0.0;
                        for (int j=0;j<view.rows;j+=sampling) {
                            for (int i=0;i<view.cols;i+=sampling) {
                                avg += view(j,i);
                                ++ count;
                            }
                        }
                        avg /= count;
                        exposure += gain * (128-avg);
                        break;
                    }
                default:
                    ROS_ERROR("Control type %d not implemented",ctrl_type);
                    return;
            }
            // Saturation
            if (exposure > 10000) exposure = 10000;
            if (exposure < 0) exposure = 0;

            axis_camera::Axis axis;
            axis.fields = axis_camera::Axis::SELECT_BRIGHTNESS;
            axis.brightness = exposure;
            axis_pub_.publish(axis);
        }
};


int main(int argc, char *argv[]) {
    ros::init(argc,argv,"axis_exposure_control");

    AxisExposureControl fm;

    ros::spin();

    return 0;
}
        

