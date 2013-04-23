#!/usr/bin/python

import roslib
roslib.load_manifest('sift_detect_holo')

import sys
import os

import rospy
import sensor_msgs.msg
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, TwistStamped
from geometry_msgs.msg import PoseStamped, PointStamped
from sensor_msgs.msg import CameraInfo
from sift_detect.msg import sift_keypoints_array
from sift_detect.msg import sift_keypoint
from std_msgs.msg import Time

from random import randint
from random import choice

import tf


from math import atan2, sqrt

import cv
import cv2
import numpy as np
import itertools
from cv_bridge import CvBridge

from pylab import imread, imshow, gray, mean


from pylab import *



class VisualServoOutliers:
	def __init__(self):

		rospy.init_node('img_detect')
		self.scale = rospy.get_param("~scale_servo",1)
		self.twist_pub = rospy.Publisher("~twist",Twist)
		rospy.Subscriber("~info",CameraInfo,self.store_info)
		rospy.Subscriber("~kp", sift_keypoints_array, self.visual_servo)
		
		self.Time_pub = rospy.Publisher("~compTime", Time)
		self.computationalTime = 0
		
		self.listener = tf.TransformListener()
		self.vel_pub = rospy.Publisher("~vel",Twist)
		
		self.f = 759.3 
		self.xc = 370.91 
		self.yc = 250.9 
		
		self.ok_model = 0
		self.vel = []

		self.cameraFrame = rospy.get_param("~camFrame","/frontCamera")
		self.robotFrame = rospy.get_param("~robFrame","/rosControlledBubbleRob")
		
	def cumputeCommand(self,kPoints):
	    
	    
	    nKP = len(kPoints.tkp)
	    
	    error = np.asmatrix(np.zeros((2*nKP,1)))
	    
	    L = np.asmatrix(np.zeros((2*nKP,3)))
	    
	    for i in range(nKP):
	        
	        
	        #print ("-------------- i="+str(i))
	        #filling up the interaction matrix
	        
	        x = (-kPoints.tkp[i].x + self.xc)/self.f
	        y = (-kPoints.tkp[i].y + self.yc)/self.f
	        
	        L[2*i,0] = -1
	        L[2*i,1] = x
	        L[2*i,2] = -(1+x*x)
	        
	        L[2*i+1,0] = 0
	        L[2*i+1,1] = y
	        L[2*i+1,2] = -x*y
	        
	        #L[2*i,0] = -1
	        #L[2*i,1] = 0
	        #L[2*i,2] = x
	        #L[2*i,3] = x*y
	        #L[2*i,4] = -(1+x*x)
	        #L[2*i,5] = y
	        
	        #L[2*i+1,0] = 0z
	        #L[2*i+1,1] = -1
	        #L[2*i+1,2] = y
	        #L[2*i+1,3] = 1 + y*y
	        #L[2*i+1,4] = -x*y
	        #L[2*i+1,5] = -x
	        
	        #computing the error matrix
	        
	        error[2*i,0]=(-kPoints.tkp[i].x + self.xc)/self.f - (-kPoints.skp[i].x + self.xc)/self.f
	        error[2*i+1,0]=(-kPoints.tkp[i].y + self.yc)/self.f - (-kPoints.skp[i].y + self.yc)/self.f
	        
	    L_pi = linalg.pinv(L)
	    
	    vel = -self.scale * L_pi*error
	    return(-self.scale * L_pi*error)
        
	def visual_servo(self,kPoints):
				
		#kPoints.skp=[sift_keypoint(0,0), sift_keypoint(0,10), sift_keypoint(10,0)]
		#kPoints.tkp=[sift_keypoint(0,0), sift_keypoint(0,10), sift_keypoint(5,0)]
		self.computationalTime = rospy.Time.now()
		
		nKP = len(kPoints.tkp)
		#print("************ len nkp = "+str(nKP))
		
		Np = 3;
		
		if(nKP > Np):
		    self.ok_model = 0
		    nTimes = 0
		
		    while self.ok_model == 0 and nTimes < nKP:
		        
		        nInliers = Np
		        
		        data_ind = range(nKP)
		        
		        ###############INIT#####################
		        
		        
		        #if(Np==0):
		        #    Np=1
		        
		        #print("number of points for init = ", Np)
		        
		        kp_array_init = sift_keypoints_array()
		        kp_array_init.header = kPoints.header

		        for p in range(Np):
		            ind = choice(data_ind)
		            kp_array_init.skp.append(kPoints.skp[ind])
		            kp_array_init.tkp.append(kPoints.tkp[ind]) 
		            data_ind.remove(ind)
		            #print("data_ind",data_ind)
		            
		        
		        
		        vel_init = self.cumputeCommand(kp_array_init)
		        
		        ################END_INIT##########################
		        

		        while len(data_ind) >= Np:
		            kp_array = sift_keypoints_array()
		            kp_array.header = kPoints.header
		            for p in range(Np):
		                ind = choice(data_ind)
		                kp_array.skp.append(kPoints.skp[ind])
		                kp_array.tkp.append(kPoints.tkp[ind]) 
		                data_ind.remove(ind)
		                #print("data_ind",data_ind)
		            vel_to_compare = self.cumputeCommand(kp_array)
		            
		            #print("vel init = (%f,%f,%f)" %(vel_init[0,0], vel_init[1,0], vel_init[2,0]))
		            #print("vel to compare = (%f,%f,%f)" %(vel_to_compare[0,0], vel_to_compare[1,0], vel_to_compare[2,0]))
		            #print("angle init = %f, angle to compare = %f" %(atan2(vel_init[1,0],vel_init[0,0]), atan2(vel_to_compare[1,0],vel_to_compare[0,0])) )
		            if(abs(atan2(vel_to_compare[1,0],vel_to_compare[0,0]) - atan2(vel_init[1,0],vel_init[0,0])) < 0.7 and vel_to_compare[2,0]*vel_init[2,0] > 0):
		            
		                #print("!!!ensemble coherent!!!")
		                nInliers += Np
		        
		          
		        if nInliers*1.0/len(kPoints.tkp) > 0.7:
		            print("!!!model ok!!! pourcent of points = %f after %d times for %d points" %(nInliers*1.0/len(kPoints.tkp), nTimes+1, nKP))
		            self.ok_model = 1
		            self.vel = vel_init
		            
		        nTimes += 1
		        
		        
		        ################END_REESTIM##########################
		        
		        
		    if(self.ok_model == 0):
		    	print("!!!! no model found !!!!")
		    vel = self.vel
		    print("vel = ",vel)
		    
		    t = Twist()
		    
		    #vc = [vx vz wy] = [ros.vy ros.vx ros.wz]
		    
		    # transform the linear velocities from the camera frame to the robot frame
		    
		    v = PointStamped()
		    v.header = kPoints.header
		    v.point.x = vel[0,0]
		    v.point.y = 0
		    v.point.z = vel[1,0]
		    
		    ((x,y,_),rot) = self.listener.lookupTransform(self.robotFrame,self.cameraFrame, rospy.Time(0))
		    
		    self.listener.waitForTransform(self.cameraFrame, self.robotFrame, kPoints.header.stamp, rospy.Duration(1.0))
		    v = self.listener.transformPoint(self.robotFrame,v)
		    
		    v.point.x = v.point.x-x
		    v.point.y = v.point.y-y
		    
		    #the rotation is also brought back to the z axis  (for the command to be correct for the robot, we add a - sign to the z axis)
		    
		    t.linear.x = v.point.x
		    t.linear.y = v.point.y
		    t.linear.z = 0
		    t.angular.x = 0
		    t.angular.y = 0
		    t.angular.z = -vel[2,0]
		    
		    self.twist_pub.publish(t)
		    self.computationalTime = rospy.Time.now() - self.computationalTime
		    self.Time_pub.publish(self.computationalTime)
	        
	def store_info(self,info):
		self.f = 759.3 #info.K[0]
		self.xc = 370.91 #info.K[2]
		self.yc = 250.9 #info.K[5]
		
		#print("Got camera info: f %.2f C %.2f %.2f" % (self.f,self.xc,self.yc))
		

	def run(self):
		rospy.loginfo("Starting visual servo")
		rospy.sleep(1.0)
		rospy.spin()
		
		

if __name__ == '__main__':

	demo = VisualServoOutliers()
	demo.run()	
