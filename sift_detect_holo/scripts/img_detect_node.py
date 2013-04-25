#!/usr/bin/python

import roslib
roslib.load_manifest('sift_detect_holo')

import sys
import os

import rospy
import sensor_msgs.msg
from sensor_msgs.msg import Image
from sift_detect.msg import sift_keypoints_array
from sift_detect.msg import sift_keypoint
from axis_camera.msg import Axis
from std_msgs.msg import Time

from dynamic_reconfigure.server import Server
from sift_detect_holo.cfg import SiftDescriptConfig

import cv
import cv2
import numpy as np
import itertools
from cv_bridge import CvBridge

from pylab import imread, imshow, gray, mean


from pylab import *

class SIFT:
	def __init__(self):
	
		rospy.init_node('sift_detect')
		
		rospy.loginfo("init node")
		self.threshold = rospy.get_param("~threshold",1)
		self.kp_pub = rospy.Publisher("~kp", sift_keypoints_array)
		self.command_cam_pub = rospy.Publisher("~command_cam", Axis)
		self.Time_pub = rospy.Publisher("~compTime", Time)
		rospy.loginfo("init node")
		
		self.reconfig_srv = Server(SiftDescriptConfig, self.reconfig_cb)
		rospy.sleep(1.0)
		rospy.loginfo("init node")
		
		
		self.computationalTime = 0
		rospy.Subscriber("~visionSensor", sensor_msgs.msg.Image, self.detect_and_draw)
		rospy.loginfo("subscribed to visionSensor")
		
		self.init = 1 
		

	def reconfig_cb(self, config, level):
		self.threshold = config["siftThreshold"]
		print ("Threshold Reconfigured ")
		return config


	

	def detect_and_draw(self, imgmsg):
		
		
		self.computationalTime = rospy.Time.now()
		#print 'number of KeyPoint objects skp', len(self.skp)
		
		
		br = CvBridge()
		temp = br.imgmsg_to_cv(imgmsg, "bgr8")
		
		im_height = temp.height
		im_length = temp.width
	
		cv.ShowImage("view",temp)
	
		cv.WaitKey(10)
		template = np.asarray(temp)
	
	
		tkp = self.detector.detect(template)
		tkp,td = self.descriptor.compute(template, tkp)
		
		if (self.init == 1):
			self.im_ref = template
			self.skp = tkp
			self.sd = td
			self.init = 0
			command_cam = Axis()
			command_cam.pan = 0
			command_cam.tilt = 0
			command_cam.zoom = 1
			self.command_cam_pub.publish(command_cam);
		
		
	
		#print 'number of KeyPoint objects tkp', len(tkp)
		#print 'number of KeyPoint objects skp', len(self.skp)
	
		flann_params = dict(algorithm=1, trees=4)
		flann = cv2.flann_Index(self.sd, flann_params)
		idx, dist = flann.knnSearch(td, 1, params={})
		del flann

		dist = dist[:,0]/2500.0
		dist = dist.reshape(-1,).tolist()
		idx = idx.reshape(-1).tolist()
		indices = range(len(dist))
		indices.sort(key=lambda i: dist[i])
		dist = [dist[i] for i in indices]
		idx = [idx[i] for i in indices]

		h1, w1 = self.im_ref.shape[:2]
		h2, w2 = template.shape[:2]
		

		skp_final = []
		for i, dis in itertools.izip(idx, dist):
			if dis < self.threshold and self.skp[i].pt[1]*1.0 < 5*h1/6.0:
				skp_final.append(self.skp[i])
			else:
				break
	
			
		tkp_final = []
		for i, dis in itertools.izip(range(len(idx)), dist):
			if dis < self.threshold and self.skp[idx[i]].pt[1]*1.0 < 4*h1/6.0:
				tkp_final.append(tkp[indices[i]])
			else:
				break
		
		
		nWidth = w1+w2
		nHeight = max(h1, h2)
		hdif = (h1-h2)/2
		newimg = np.zeros((nHeight, nWidth, 3), np.uint8)
		newimg[hdif:hdif+h2, :w2] = template
		newimg[:h1, w2:w1+w2] = self.im_ref

		tkp_final
		skp_final

		#print 'number of KeyPoint objects in skp_final', len(skp_final)
		#print 'number of KeyPoint objects in tkp_final', len(tkp_final)

		for i in range(min(len(tkp_final), len(skp_final))):
			
			pt_a = (int(tkp_final[i].pt[0]), int(tkp_final[i].pt[1]+hdif))
			pt_b = (int(skp_final[i].pt[0]+w2), int(skp_final[i].pt[1]))
	
			cv2.circle(newimg, pt_a, int(tkp_final[i].size),(160,32,240),1)
			cv2.circle(newimg, pt_b, int(skp_final[i].size),(160,32,240),1)
			cv2.line(newimg, pt_a, pt_b, (255, 0, 0))
	
			cv.ShowImage("sift",cv.fromarray(newimg))
		
		kp_array = sift_keypoints_array()
		#kp_array.header = imgmsg.header
		kp_array.header.frame_id = imgmsg.header.frame_id
		kp_array.header.stamp = rospy.Time(0) #Time().now()
		kp_array.skp = [sift_keypoint(*k.pt) for k in skp_final]
		kp_array.tkp = [sift_keypoint(*k.pt) for k in tkp_final]

			
		
		self.kp_pub.publish(kp_array)
		self.computationalTime = rospy.Time.now() - self.computationalTime
		self.Time_pub.publish(self.computationalTime)
		
		key=cv.WaitKey(10) & 0xFF
		
		if key == ord('d'):
			self.im_ref = template
			self.skp = tkp
			self.sd = td
			

	def run(self):
	
		pkgdir = roslib.packages.get_pkg_dir("opencv2")

		dirname, filename = os.path.split(os.path.abspath(__file__))

		#print "path",os.path

		path_imref = ""+dirname+"/../ROS/HeronLorraine.jpg"

		#self.im_ref = cv2.imread(path_imref)

		#if(self.im_ref == None):
		#	print "image vide"
	
		self.detector = cv2.FeatureDetector_create("SIFT")
		self.descriptor = cv2.DescriptorExtractor_create("SIFT")

		#self.skp = self.detector.detect(self.im_ref)
		#self.skp,self.sd = self.descriptor.compute(self.im_ref, self.skp)
	
	
		#print 'number of KeyPoint objects skp', len(self.skp)
		
		rospy.spin()


if __name__ == '__main__':

	node = SIFT()
	node.run()

