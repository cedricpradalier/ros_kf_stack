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

# Follow shore until the condition get triggered
tc.FollowShoreRP(velocity=0.4, distance=10.0, side=-1, k_alpha=1.0, max_ang_vel=0.7, velocity_scaling=0.4)

rospy.loginfo("Mission completed")

