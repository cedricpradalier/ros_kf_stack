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

try:
    for angle in [pi/4, 0.,pi/2,pi,3*pi/2,0.]:
        tc.SetPTZ(tilt=0.,pan=angle,zoom=1.,wait_timeout=15.0,max_command_rate=0.5)
        tc.Wait(duration=2.0)

except TaskException, e:
    rospy.logerr("Exception caught: " + str(e))


rospy.loginfo("Mission completed")


