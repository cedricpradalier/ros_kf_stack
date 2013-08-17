#!/usr/bin/env python
import roslib; roslib.load_manifest('kf_yaw_kf')
import rospy
import math
from math import pi
from geometry_msgs.msg import Vector3Stamped,Vector3
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from kf_yaw_kf.msg import Compass
from kf_yaw_kf.srv import *
from numpy import *
from numpy.linalg import inv
from threading import Lock
import message_filters

def norm_angle(x):
    return ((x + pi) % (2*pi))-pi 


class KFYawKF:
    def __init__(self):
        self.X = zeros((4,1))
        self.P = eye(4)
        self.first = True
        self.mag_x_offset = 0
        self.mag_y_offset = 0
        self.mag_z_offset = 0
        self.compass = Compass()
        self.mutex = Lock()

    def set_mag_offset(self,req):
        with self.mutex:
            self.mag_x_offset = req.mag_x_offset
            self.mag_y_offset = req.mag_y_offset
            self.mag_z_offset = req.mag_z_offset
            self.X = zeros((4,1))
            self.P = eye(4)
            self.first = True
            rospy.loginfo("Updated Magnetometer offset to %.2f %.2f %.2f" % (self.mag_x_offset,self.mag_y_offset,self.mag_z_offset))
        return SetMagOffsetResponse()
        
    def ready(self):
        return (not self.first_rpy) and (not self.first_imu) and (not self.first_mag)

    def kf_update(self, Z, H, R):
        with self.mutex:
            # print "-----"
            I = Z - H * self.X
            I[0,0] = norm_angle(I[0,0])
            I[1,0] = norm_angle(I[1,0])
            K = self.P * H.T * inv(H * self.P * H.T + R)
            # print self.X
            # print H
            # print I
            self.X = self.X + K * I
            self.X[0,0] = norm_angle(self.X[0,0])
            self.P = (eye(4) - K * H) * self.P

    def kf_predict(self, dt, Q):
        with self.mutex:
            # X = [phi_i, phi_{i-1}, omega, omega_bias]
            A = zeros((4,4))
            A[0,0] = 1; A[0,1] = dt;
            A[1,0] = 1;
            A[2,2] = 1;
            A[2,2] = 1;
            self.X = A * self.X
            self.X[0,0] = norm_angle(self.X[0,0])
            self.P = A * self.P * A.T + Q

    def run_kf(self,*arg):
        if len(arg)!=3:
            rospy.logerr("Invalid number of argument in run_kf")
            return
        now = rospy.Time.now()
        omega = arg[0].angular_velocity.z
        phi_mag = math.atan2(-(arg[2].vector.x-self.mag_x_offset),-(arg[2].vector.y-self.mag_y_offset))
        self.mag_pub.publish(Float32(norm_angle(pi/2 - phi_mag)))
        if not self.first:
            dt = (now - self.last_stamp).to_sec()
            self.kf_predict(dt,self.Q) 
            dphi_gyro = -norm_angle(arg[1].vector.z - self.last_phi_gyro)
            Z = mat([phi_mag, dphi_gyro, omega]).transpose()
            H = mat([[1,0,0,0],[1,-1,0,dt],[0,0,1,0]])
            self.kf_update(Z,H,self.R)
            self.compass.heading = self.X[0,0]
            self.compass.compass = norm_angle(pi/2 - self.compass.heading)
            self.compass.stddev = math.sqrt(self.P[0,0])
            if self.replay:
                self.compass.header.stamp = now
            else:
                self.compass.header.stamp = arg[1].header.stamp
            self.compass_pub.publish(self.compass)
            if self.debug_pub:
                self.q_pub.publish(Vector3(self.X[0,0], self.X[1,0], self.X[2,0]))
                self.p_pub.publish(Vector3(self.P[0,0], self.P[1,1], self.P[2,2]))
        else:
            self.X[0,0] = phi_mag

        self.last_phi_gyro = arg[1].vector.z
        self.last_stamp = now;
        self.first = False


    def run(self):
        rospy.init_node("kf")
        self.first = True
        self.frame_id = rospy.get_param("~frame_id","/kingfisher/base")
        self.stddev_yaw_mag = rospy.get_param("~stddev_yaw_mag",0.1)
        self.stddev_yaw_gyro = rospy.get_param("~stddev_yaw_gyro",0.01)
        self.stddev_omega = rospy.get_param("~stddev_omega",0.01)
        self.mag_x_offset = rospy.get_param("~mag_offset_x",0.0)
        self.mag_y_offset = rospy.get_param("~mag_offset_y",0.0)
        self.mag_z_offset = rospy.get_param("~mag_offset_z",0.0)
        self.Q = mat(diag([1e-3,1e-5,1e-3,1e-5]))
        self.R = diag([self.stddev_yaw_mag*self.stddev_yaw_mag,self.stddev_yaw_gyro*self.stddev_yaw_gyro,self.stddev_omega*self.stddev_omega])
        self.debug_pub = rospy.get_param("~debug_publishers",True)
        # necessary to set fake time stamp when replaying
        self.replay = rospy.get_param("~replay",True)
        self.mag_pub = rospy.Publisher("~mag",Float32) # Raw magnetic compass output
        if self.replay:
            self.compass_pub = rospy.Publisher("~compass2",Compass)
        else:
            self.compass_pub = rospy.Publisher("~compass",Compass)
        if self.debug_pub:
            self.q_pub = rospy.Publisher("~state",Vector3) # Convenience debug output
            self.p_pub = rospy.Publisher("~cov",Vector3) # Convenience debug output
        self.compass.header.frame_id = self.frame_id
        self.imu_sub = message_filters.Subscriber("/imu/data",Imu)
        self.rpy_sub = message_filters.Subscriber("/imu/rpy",Vector3Stamped)
        self.mag_sub = message_filters.Subscriber("/imu/mag",Vector3Stamped)
        self.ts = message_filters.TimeSynchronizer([self.imu_sub,self.rpy_sub,self.mag_sub], 10)
        self.mag_offset_service = rospy.Service('~mag_offset', SetMagOffset, self.set_mag_offset)
        self.ts.registerCallback(self.run_kf)
        rospy.loginfo("Compass up and running")
        rospy.spin()



if __name__ == '__main__':
    try:
        kf = KFYawKF()
        kf.run()
    except rospy.ROSInterruptException:
        pass
