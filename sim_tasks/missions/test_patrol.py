#!/usr/bin/python
# ROS specific imports
import roslib; roslib.load_manifest('sim_tasks')
import rospy
from math import *
from task_manager_lib.TaskClient import *

rospy.init_node('task_client')
server_node = rospy.get_param("~server","/sim_tasks")
default_period = rospy.get_param("~period",0.05)
tc = TaskClient(server_node,default_period)
rospy.loginfo("Mission connected to server: " + server_node)

tc.WaitForAuto()
try:
	tc.AlignWithShore(angle=1.57, ang_velocity=1.5)
	tc.RecordFinishLine()
	while True:
		try:		
			tc.FollowShorePID(angle=1.57, velocity=0.2, p_d=0.25, p_alpha=0.3, distance=8.0,task_timeout=-1.0)
		except TaskException, e:
			pass
		try:
			tc.FollowShorePID(angle=-1.57, velocity=0.2, p_d=0.25, p_alpha=0.3, distance=8.0,task_timeout=-1.0)
		except TaskException, e:
			pass

except TaskException, e:
    rospy.logerr("Exception caught: " + str(e))

if not rospy.core.is_shutdown():
    tc.SetManual()

rospy.loginfo("Mission completed")

