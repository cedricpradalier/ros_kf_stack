#include "ros/ros.h"
#include <sstream>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/PointStamped.h"
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

class GpsSimul {
public:
  GpsSimul();

protected:
  float randomNumber();

  ros::NodeHandle n;

  tf::TransformListener listener;
  ros::Publisher pubUtm;
  ros::Publisher pubCompass;
  ros::Publisher pubUtmNoise;
  ros::Publisher pubCompassNoise;
};

GpsSimul::GpsSimul() {

  srand(time(NULL));
  ros::Rate loop_rate(10);

  pubUtm = n.advertise<geometry_msgs::PointStamped>("/gpsnode/utm",1);
  pubCompass = n.advertise<std_msgs::Float64>("/gpsnode/compass",1);
  pubUtmNoise = n.advertise<geometry_msgs::PointStamped>("/gpsnode/utmNoise",1);
  pubCompassNoise = n.advertise<std_msgs::Float64>("/gpsnode/compassNoise",1);

  geometry_msgs::PointStamped utmValue;
  std_msgs::Float64 compassValue;
  geometry_msgs::PointStamped utmNoiseValue;
  std_msgs::Float64 compassNoiseValue;

  float u_sum = 0;
  float standard_deviation_G = 0.1;
  float standard_deviation_RW = 5.0;

  while (ros::ok()) {
    
    tf::StampedTransform transform;
    try {
      listener.waitForTransform("/world","/rosControlledBubbleRob", ros::Time(0), ros::Duration(5.0));
      listener.lookupTransform("/world","/rosControlledBubbleRob", ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }

    utmValue.point.x = transform.getOrigin().x() + 692124.07;
    utmValue.point.y = transform.getOrigin().y() + 6292521.8;
    utmValue.point.z = 0.0;
    compassValue.data = tf::getYaw(transform.getRotation());

    // Random-Walk noise
    u_sum += randomNumber();
    utmNoiseValue.point.x = utmValue.point.x + standard_deviation_RW * cos(u_sum);
    utmNoiseValue.point.y = utmValue.point.y + standard_deviation_RW * sin(u_sum);
    utmNoiseValue.point.z = utmValue.point.z;

    // Gaussian noise
    compassNoiseValue.data = compassValue.data + standard_deviation_G * sqrt(-2*log(randomNumber())) * cos(2*M_PI*randomNumber());

    ROS_INFO("utmValue: x %.3f / y %.3f",utmValue.point.x,utmValue.point.y);
    ROS_INFO("utmNoiseValue: x %.3f / y %.3f",utmNoiseValue.point.x,utmNoiseValue.point.y);
    ROS_INFO("compassValue %.3f",compassValue.data);
    ROS_INFO("compassNoiseValue %.3f",compassNoiseValue.data);

    pubUtm.publish(utmValue);
    pubCompass.publish(compassValue);
    pubUtmNoise.publish(utmNoiseValue);
    pubCompassNoise.publish(compassNoiseValue);

    loop_rate.sleep();
  }
}

float GpsSimul::randomNumber() {
  // Uniform distribution
  return rand()/(double)RAND_MAX;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gpsnode");
  GpsSimul gpssimul;

  ros::spin();
}

