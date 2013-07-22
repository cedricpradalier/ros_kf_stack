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
tc.AlignWithShore(angle=1.57, ang_velocity=1.5)
w4roi = tc.WaitForROI(foreground=False,current=True,
        roi_radius=2.0, histeresis_radius=1.0)
w4dist = tc.WaitForDistance(foreground=False,distance=5.0)
tc.addCondition(ConditionIsCompleted("Distance",tc,w4dist))
try:
    # Follow shore until the condition get triggered
    tc.FollowShoreRP(velocity=0.5, distance=6.0, side=+1, k_alpha=1.0)
except TaskConditionException, e:
    pass

# Now turn around and come back
tc.AlignWithShore(angle=-1.57, ang_velocity=1.5)
tc.addCondition(ConditionIsCompleted("ROI detector",tc,w4roi))
try:
    # Follow shore until the condition get triggered
    tc.FollowShoreRP(velocity=0.5, distance=6.0, side=-1, k_alpha=1.0)
except TaskConditionException, e:
    pass

if not rospy.core.is_shutdown():
    tc.SetManual()

rospy.loginfo("Mission completed")

