#!/usr/bin/python
# ROS specific imports
import roslib; roslib.load_manifest('sim_tasks')
import rospy
from math import *
from task_manager_lib.TaskClient import *

rospy.init_node('task_client')
server_node = rospy.get_param("~server","/task_server")
default_period = rospy.get_param("~period",0.05)
tc = TaskClient(server_node,default_period)
rospy.loginfo("Mission connected to server: " + server_node)

tc.WaitForAuto()
try:
    for contour in range(0,2):
        tc.AlignWithShore(angle=1.57, ang_velocity=0.5)
        tc.RecordFinishLine()
	tc.FollowShorePID(angle=1.57, velocity=0.3, p_d=0.15, p_alpha=0.25,  d_alpha=-0.15, d_d=-0.15, i_d=0.01, i_alpha=0.01, distance=3.0)
        tc.ReachOtherShore(angle=1.57)

except TaskException, e:
    rospy.logerr("Exception caught: " + str(e))

if not rospy.core.is_shutdown():
    tc.SetManual()


rospy.loginfo("Mission completed")
while not rospy.core.is_shutdown():
	rospy.sleep(1)

