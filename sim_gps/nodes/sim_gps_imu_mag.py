#!/usr/bin/env python
import roslib; roslib.load_manifest('sim_gps')
import rospy
import math
from math import pi
import random
from geometry_msgs.msg import Vector3Stamped, Quaternion
from sensor_msgs.msg import Imu
from gps_common.msg import GPSFix


def norm_angle(x):
    return ((x + pi) % (2*pi))-pi 

class SimGpsImuMag:
    def __init__(self):
        self.dt = 0.050 # 20Hz
        self.theta = 0
        self.noise_gps = 0.1
        self.delta_mag = 0.5
        self.noise_mag = 0.2
        self.noise_magnitude_mag = 20.0
        self.bias_index = 0.0
        self.bias_yaw = -2.0
        self.noise_yaw = 0.01
        self.bias_speed = 1e-3 / self.dt
        self.omega = -0.1
        self.noise_omega = 0.01

        rospy.init_node("sim_theta")
        self.gps_pub = rospy.Publisher("/gps/extfix",GPSFix)
        self.rpy_pub = rospy.Publisher("/imu/rpy",Vector3Stamped)
        self.imu_pub = rospy.Publisher("/imu/data",Imu)
        self.mag_pub = rospy.Publisher("/imu/mag",Vector3Stamped)
        self.q_pub = rospy.Publisher("/sim/q",Quaternion)

    def noise(self,value):
        return (-1+2*random.random())*value

    def run(self):
        counter = 0
        rospy.loginfo("Simulation started")
        while not rospy.is_shutdown():
            self.q_pub.publish(Quaternion(self.theta,self.omega,self.bias_yaw,self.delta_mag))

            now = rospy.Time.now()
            rpy = Vector3Stamped()
            rpy.header.stamp = now
            rpy.vector.z = self.theta - self.bias_yaw + self.noise(self.noise_yaw)
            self.rpy_pub.publish(rpy)

            mag = Vector3Stamped()
            mag.header = rpy.header
            magnitude = 150 + self.noise(self.noise_magnitude_mag)
            theta = self.theta - self.delta_mag + self.noise(self.noise_mag)
            mag.vector.x = magnitude*math.cos(theta)
            mag.vector.y = magnitude*math.sin(theta)
            self.mag_pub.publish(mag)

            imu = Imu()
            imu.header = mag.header
            imu.angular_velocity.z = self.omega + self.noise(self.noise_omega)
            self.imu_pub.publish(imu)
            
            if (counter % 20) == 0:
                gps = GPSFix()
                gps.header = rpy.header
                gps.speed = 1.0
                gps.track = self.theta + self.noise(self.noise_gps)
                gps.track = norm_angle(gps.track)
                self.gps_pub.publish(gps)

            counter += 1
            self.bias_index += self.bias_speed * self.dt
            self.bias_yaw = 2*math.pi*math.cos(self.bias_index)
            self.theta += self.omega * self.dt
            self.theta = norm_angle(self.theta)
            rospy.sleep(self.dt)


if __name__ == '__main__':
    try:
        sim = SimGpsImuMag() 
        sim.run()
    except rospy.ROSInterruptException:
        pass
