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

tc.WaitForAuto()
try:
    tc.FollowShore(angle=1.57,goal_x=14.0,goal_y=-8.0)
    tc.FollowShore(angle=1.57,goal_x=-11.0,goal_y=3.0)
    tc.GoTo(goal_x=-3.2,goal_y=4.0)
    tc.FollowShore(angle=1.57,goal_x=11.5,goal_y=4.0)
    tc.FollowShore(angle=1.57,goal_x=-3.2,goal_y=4.0)

except TaskException, e:
    rospy.logerr("Exception caught: " + str(e))

if not rospy.core.is_shutdown():
    tc.SetManual()


rospy.loginfo("Mission completed")


