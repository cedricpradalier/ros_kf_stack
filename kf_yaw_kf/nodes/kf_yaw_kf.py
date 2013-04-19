#!/usr/bin/env python
import roslib; roslib.load_manifest('kf_yaw_kf')
import rospy
import math
from math import pi
from geometry_msgs.msg import Vector3Stamped,Quaternion
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from gps_common.msg import GPSFix
from numpy import *
from numpy.linalg import inv
from threading import Lock

def norm_angle(x):
    return ((x + pi) % (2*pi))-pi 


class KFYawKF:
    def __init__(self):
        self.X = zeros((4,1))
        self.P = eye(4)
        self.first_rpy = True
        self.first_gps = True
        self.first_mag = True
        self.first_imu = True
        self.mutex = Lock()
        
    def ready(self):
        return (not self.first_rpy) and (not self.first_imu) and (not self.first_mag) and (not self.first_gps)

    def kf_update(self, Z, H, R, angle=False, pub=None):
        with self.mutex:
            # print "-----"
            I = Z - H * self.X
            if angle:
                I = norm_angle(I)
            if pub:
                pub.publish(I[0,0])
            K = self.P * H.T * inv(H * self.P * H.T + R)
            # print self.X
            # print H
            # print I
            self.X = self.X + K * I
            self.X[0,0] = norm_angle(self.X[0,0])
            self.P = (eye(4) - K * H) * self.P
            self.q_pub.publish(Quaternion(self.X[0,0], self.X[1,0], self.X[2,0], self.X[3,0]))

    def kf_predict(self, dt, Q):
        with self.mutex:
            A = eye(4)
            A[0,1] = dt
            self.X = A * self.X
            self.X[0,0] = norm_angle(self.X[0,0])
            self.P = A * self.P * A.T + Q
            self.q_pub.publish(Quaternion(self.X[0,0], self.X[1,0], self.X[2,0], self.X[3,0]))

    def gps_cb(self,data):
        if data.speed < 0.3:
            return 
        yaw_gps = data.track
        self.gps_pub.publish(Float32(yaw_gps))
        if self.first_gps:
            self.X[2,0] = yaw_gps
            rospy.loginfo("Initialised main state")
            self.first_gps = False
            return 
        Z = mat([yaw_gps])
        H = mat([1,0,0,0])
        R = max(0.3, 1.0 - (data.speed - 0.5)*0.7)
        R = mat([R])
        self.kf_update(Z,H,R,True,self.egps_pub)

    def mag_cb(self,data):
        # TODO: Integrate roll and pitch...
        yaw_mag = math.atan2(data.vector.y,data.vector.x)
        self.mag_pub.publish(Float32(yaw_mag))
        if self.first_mag and not self.first_gps:
            self.X[2,0] = self.X[0,0] - yaw_mag
            rospy.loginfo("Initialised MAG bias")
            self.first_mag = False
            return 
        if self.first_mag:
            return
        Z = mat([yaw_mag])
        H = mat([1,0,0,-1])
        R = mat([0.1*0.1])
        self.kf_update(Z,H,R,True,self.emag_pub)

    def rpy_cb(self,data):
        yaw_imu = -data.vector.z
        self.yaw_pub.publish(Float32(yaw_imu))
        if self.first_rpy and not self.first_mag:
            self.X[2,0] = self.X[0,0] - yaw_imu
            rospy.loginfo("Initialised IMU bias")
            self.first_rpy = False
            return 
        if self.first_rpy:
            return
        Z = mat([yaw_imu])
        H = mat([1,0,-1,0])
        R = mat([0.1*0.1])
        self.kf_update(Z,H,R,True,self.erpy_pub)


    def imu_cb(self,data):
        omega = -data.angular_velocity.z
        self.first_imu = False
        if self.ready():
            Z = mat([omega])
            H = mat([0,1,0,0])
            R = mat([0.2*0.2])
            self.kf_update(Z,H,R,False)


    def run(self):
        rospy.init_node("kf")
        self.gps_sub = rospy.Subscriber("/gps/extfix",GPSFix,self.gps_cb)
        self.imu_sub = rospy.Subscriber("/imu/data",Imu,self.imu_cb)
        self.rpy_sub = rospy.Subscriber("/imu/rpy",Vector3Stamped,self.rpy_cb)
        self.mag_sub = rospy.Subscriber("/imu/mag",Vector3Stamped,self.mag_cb)
        self.q_pub = rospy.Publisher("~state",Quaternion) # Convenience debug output
        self.emag_pub = rospy.Publisher("~emag",Float32) # Convenience debug output
        self.erpy_pub = rospy.Publisher("~erpy",Float32) # Convenience debug output
        self.egps_pub = rospy.Publisher("~egps",Float32) # Convenience debug output
        self.yaw_pub = rospy.Publisher("~yaw",Float32) # Convenience debug output
        self.mag_pub = rospy.Publisher("~mag",Float32) # Convenience debug output
        self.gps_pub = rospy.Publisher("~gps",Float32) # Convenience debug output
        rate = rospy.Rate(10)
        Q = mat(diag([1e-5,1e-5,1e-3,1e-8]))
        rospy.loginfo("Ready to estimate yaw angle")
        while not rospy.is_shutdown():
            if self.ready():
                self.kf_predict(rate.sleep_dur.to_sec(),Q)
            rate.sleep()



if __name__ == '__main__':
    try:
        kf = KFYawKF()
        kf.run()
    except rospy.ROSInterruptException:
        pass
