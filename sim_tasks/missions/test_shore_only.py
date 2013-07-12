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
tc.RecordFinishLine()
tc.SetPTZ(pan=1.57, tilt=0.0)
tc.AlignWithShore(angle=1.57, ang_velocity=1.5)
tc.FollowShorePID(angle=1.57, velocity=0.3, p_d=0.15, p_alpha=0.3,  d_alpha=-0.1, d_d=-0.1, i_d=0.01, i_alpha=0.01, distance=8.0, task_timeout=-1.0)

except TaskException, e:
    rospy.logerr("Exception caught: " + str(e))

if not rospy.core.is_shutdown():
    tc.SetManual()

rospy.loginfo("Mission completed")

