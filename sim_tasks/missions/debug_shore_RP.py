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



tc.FollowShoreRP(velocity=0.35, distance=10.0, side=-1, k_alpha=0.60, max_ang_vel=0.60,velocity_scaling=0.1,radial=False, \
        angular_steps=8, forward_range=10.0, backward_range=5.0, radial_resolution=0.5, \
        k_initial_angle=1.0, k_dist=1.0, k_length=0.0, k_turn=3.0)

rospy.loginfo("Mission completed")

