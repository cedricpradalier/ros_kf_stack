#!/usr/bin/python

import roslib
roslib.load_manifest('sift_detect_holo')

import sys
import os

import rospy
import sensor_msgs.msg
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, TwistStamped, Quaternion
from geometry_msgs.msg import PoseStamped, PointStamped
from sensor_msgs.msg import CameraInfo
from sift_detect.msg import sift_keypoints_array
from sift_detect.msg import sift_keypoint

from visualization_msgs.msg import Marker

import tf

from dynamic_reconfigure.server import Server
from sift_detect_holo.cfg import VelConvertConfig


from math import atan2, sqrt

import cv
import cv2
import numpy as np
import itertools
from cv_bridge import CvBridge

from pylab import imread, imshow, gray, mean


from pylab import *

reset_button = 2

def norm_angle(x):
	if x > pi:
		x -= 2*pi
	if x < -pi:
		x += 2*pi
	return x
	
	  
def sat(x, mx):
	if x > mx:
		return mx
	if x < -mx:
		return -mx
	return x


class VelConvert:
    def __init__(self):
	
        rospy.init_node('vel_def')
		
        self.scale_rotRob = rospy.get_param("~scaleRotRob",0.5)
        self.sigmaVel = rospy.get_param("~sigmaVel",0.5)
        self.minVel = rospy.get_param("~minVel",0.025)
        self.maxVel = rospy.get_param("~maxVel",0.3)
        self.minVelForRot = rospy.get_param("~minVelForRot",0.08)
        self.maxRotRob = rospy.get_param("~maxRotRob",0.5)
        self.maxRotCam = rospy.get_param("~maxRotCam",0.5)
	self.scale_lin = rospy.get_param("~scale_lin",0.5)
        self.sigmaRot = rospy.get_param("~sigmaRot",0.08)
	self.scale_rotCam = rospy.get_param("~scaleRotCam",0.5)
	self.scale_rotInit = rospy.get_param("~scaleRotInit",1)
        rospy.Subscriber("~servo", Twist, self.convert_vel)
        sub = rospy.Subscriber('~joy', Joy, self.joy_cb)

        
        self.rob_twist_pub = rospy.Publisher("~rob_twist_pub", Twist)
        self.pan_pub_fl = rospy.Publisher("~pan_pub_float",Float64)
        self.pan_pub_tw = rospy.Publisher("~pan_pub_twist",Twist)
        self.marker_pub = rospy.Publisher("~/vel_marker",Marker)
        
        self.ang_rob = 0
        self.previous_vel_rob = [0,0,0,0,0,0,0,0,0,0]
        self.reached_goal = 0
        
        self.reconfig_srv = Server(VelConvertConfig, self.reconfig_cb)
        rospy.sleep(1.0)
		
    def joy_cb(self,value):
        global reset_button
        try:
            if value.buttons[reset_button]:
                self.reached_goal = 0
                print "servoing reset"
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
                           
    def reconfig_cb(self,config, level):
        self.scale_rotRob = config["scaleRotRob"]
	self.scale_rotCam = config["scaleRotCam"]
        self.sigmaVel = config["sigmaVel"]
        self.sigmaRot = config["sigmaRot"]
        self.minVel = config["minVel"]
        self.minVelForRot = config["minRot"]
        self.max_rotRob = config["maxRotRob"]
        self.max_rotCam = config["maxRotCam"]
	self.scale_lin = config["scale_lin"]
	self.scale_rotInit = config["scaleRotInit"]
        #print "Reconfigured"
        return config
        
    def convert_vel(self, servoTwist):
		t = Twist()
		pan_vel_f = Float64()
		pan_vel_t = Twist()
		
		vx = servoTwist.linear.x
		vy = servoTwist.linear.y
		wz = servoTwist.angular.z*self.scale_rotInit
		
		
		# computing the velocity of the camera and robot
		
		#if hypot(vx,vy)<self.minVelForRot:
		#	wr = 0
		#else:
		wr = sat(atan2(vy,vx)*self.scale_rotRob, self.max_rotRob)
		
		
		#self.previous_vel_rob.pop()
		#self.previous_vel_rob.insert(0,wr)
		
		#wr = sum(self.previous_vel_rob)/10
		
		t.linear.x = sat(self.scale_lin*hypot(vx,vy)*exp(-wr*wr/(self.sigmaVel*self.sigmaVel)),self.maxVel)
		t.linear.y = 0
		t.linear.z = 0
		t.angular.x = 0
		t.angular.y = 0
		t.angular.z = -wr
		
		#if (abs(t.linear.x)<self.minVel) and (abs(t.angular.z)<self.minVel) and (self.reached_goal == 0):
		#    self.reached_goal = 1
		#    print "GOAL REACHED"
		    
		if self.reached_goal==1:
			t.linear.x=0
			wr = 0
			wz = 0
			t.angular.z = 0
			
		
		
		#print(wz)
		
		pan_vel_f.data = sat(self.scale_rotCam*(wz-wr), self.max_rotCam)
		pan_vel_t.angular.z = sat(self.scale_rotCam*(wz-wr), self.max_rotCam)
		print("wz = %f, wr = %f, wcam = %f" %(wz, wr, sat(self.scale_rotCam*(wz-wr), self.max_rotCam)))
		
		#print("wz, wr, wpan", wz, wr, pan_vel.data)

		#print("twist command from servo: vy %.2f vz %.2f wx %.2f" % (t.linear.y,t.linear.z,t.angular.x))
		

		self.pan_pub_fl.publish(pan_vel_f)
		self.pan_pub_tw.publish(pan_vel_t)
		self.rob_twist_pub.publish(t)
		
		
	
	
	
	
    def run(self):
		rospy.sleep(1.0)
		rospy.spin()
		
if __name__ == '__main__':

	node = VelConvert()
	node.run()
	
