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



def follow_until_stopped(shore_side):
    global tc
    try:
        # Follow shore until the condition get triggered
        if True:
            tc.FollowShoreRP(radial=True,velocity=0.35, distance=10.0, side=shore_side, k_alpha=0.60, max_ang_vel=0.60,velocity_scaling=0.1)
        else:
            tc.FollowShoreRP(radial=False,velocity=0.35, distance=10.0, side=1, k_alpha=0.60, max_ang_vel=0.60,velocity_scaling=0.1, \
                    angular_steps=8, forward_range=10.0, backward_range=5.0, radial_resolution=0.5, \
                    k_initial_angle=1.0, k_dist=1.0, k_length=0.0, k_turn=3.0)
    except TaskConditionException, e:
        pass

tc.CheckReadiness(logger=True)

tc.SetPTZ(pan=1.57,tilt=0.20)
tc.AlignWithShore(angle=-1.57, ang_velocity=0.5)
# Set a distance trigger.
w4dist = tc.WaitForDistance(foreground=False,distance=100.0)
tc.addCondition(ConditionIsCompleted("Distance",tc,w4dist))
follow_until_stopped(+1)

# Store the current pose as origin
tc.SetOrigin(current=True)

# add a trigger for the point between the island and the shore
w4roi = tc.WaitForROI(foreground=False,wrtOrigin=False,roi_x=296871.51303, roi_y=5442696.42175, roi_radius=15.0, histeresis_radius=20.0)
tc.addCondition(ConditionIsCompleted("ROI detector",tc,w4roi))
follow_until_stopped(+1)

# Now follow the island shore
tc.SetPTZ(pan=-1.57,tilt=0.20)
w4roi = tc.WaitForROI(foreground=False,wrtOrigin=False,roi_x=296871.51303, roi_y=5442696.42175, roi_radius=15.0, histeresis_radius=20.0)
tc.addCondition(ConditionIsCompleted("ROI detector",tc,w4roi))
follow_until_stopped(-1)

# Finally finish the run to home
tc.SetPTZ(pan=1.57,tilt=0.20)
w4roi = tc.WaitForROI(foreground=False,wrtOrigin=True,roi_x=0.0, roi_y=0.0, roi_radius=15.0, histeresis_radius=20.0)
tc.addCondition(ConditionIsCompleted("ROI detector",tc,w4roi))
follow_until_stopped(+1)

# Finally mark completion by pointing toward the shore
tc.AlignWithShore(angle=0.0, ang_velocity=0.25)

rospy.loginfo("Mission completed")

