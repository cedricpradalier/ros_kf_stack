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


scale = 10.0 # Ok for the lake
scale = -3.0 # better for VRep

tc.WaitForAuto()
tc.SetOrigin(current=True)
try:
    # 10 meter east (of origin)
    tc.GoTo(goal_x=scale,goal_y=0.0)
    tc.Wait(duration=1.0)
    # 10 meter north
    tc.GoTo(goal_x=scale,goal_y=scale)
    tc.Wait(duration=1.0)
    # 10 m west of current position
    tc.GoTo(goal_x=-scale,goal_y=0.0, relative=True)
    tc.Wait(duration=1.0)
    # 10 m south of current position
    tc.GoTo(goal_x=0.0,goal_y=-scale, relative=True)
    # We should be back home

except TaskException, e:
    rospy.logerr("Exception caught: " + str(e))

if not rospy.core.is_shutdown():
    tc.SetManual()


rospy.loginfo("Mission completed")
