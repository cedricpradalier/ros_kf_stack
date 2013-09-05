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

tc.SetPTZ(pan=1.57,tilt=0.0)
tc.AlignWithShore(angle=1.57, ang_velocity=1.0)
# Set a distance trigger.
w4dist = tc.WaitForDistance(foreground=False,distance=10.0)
tc.addCondition(ConditionIsCompleted("Distance",tc,w4dist))
try:
    # Follow shore until the condition get triggered
    tc.FollowShoreRP(velocity=0.4, distance=10.0, side=+1, k_alpha=0.5, max_ang_vel=0.6,velocity_scaling=0.6)
except TaskConditionException, e:
    pass

# The distance triggered so we set a ROI trigger to make sure that we have some overlap
# when the run completes
w4roi = tc.WaitForROI(foreground=False,current=True,
        roi_radius=10.0, histeresis_radius=5.0)
tc.addCondition(ConditionIsCompleted("ROI detector",tc,w4roi))
try:
    # Follow shore until the new condition get triggered
    tc.FollowShoreRP(velocity=0.4, distance=10.0, side=+1, k_alpha=0.5, max_ang_vel=0.6,velocity_scaling=0.6)
except TaskConditionException, e:
    pass

# Finally mark completion by pointing toward the shore
tc.AlignWithShore(angle=0.00, ang_velocity=1.0)

rospy.loginfo("Mission completed")

