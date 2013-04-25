#!/usr/bin/python
# ROS specific imports
import roslib; roslib.load_manifest('sim_tasks')
import rospy
from math import *
from task_manager_lib.TaskClient import *

rospy.init_node('task_client')
server_node = rospy.get_param("~server","/sim_tasks")
default_period = rospy.get_param("~period",0.2)
tc = TaskClient(server_node,default_period)
rospy.loginfo("Mission connected to server: " + server_node)

tc.WaitForAuto()
try:
    tc.FindFinishLine(angle=1.57, ang_velocity=1.5)
    tc.FollowShore(angle=1.57, velocity=1.5, k_d=0.2, k_alpha=0.3, distance=0.5)
    tc.ReachOtherShore(angle=1.57)
    tc.FindFinishLine(angle=1.57, ang_velocity=1.5)
    tc.FollowShore(angle=1.57, velocity=1.5, k_d=0.2, k_alpha=0.3, distance=8.0)
    tc.ReachOtherShore(angle=1.57)

except TaskException, e:
    rospy.logerr("Exception caught: " + str(e))

if not rospy.core.is_shutdown():
    tc.SetManual()


rospy.loginfo("Mission completed")
while not rospy.core.is_shutdown():
	rospy.sleep(1)

