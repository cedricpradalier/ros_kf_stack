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
        tc.FollowShoreRP(velocity=0.5, distance=10.0, side=shore_side, k_alpha=0.75, max_ang_vel=0.7,velocity_scaling=0.4)
    except TaskConditionException, e:
        pass


# Now follow the island shore
tc.SetPTZ(pan=-1.57,tilt=0.2)
w4roi = tc.WaitForROI(foreground=False,wrtOrigin=False,roi_x=296871.51303, roi_y=5442696.42175, roi_radius=10.0)
tc.addCondition(ConditionIsCompleted("ROI detector",tc,w4roi))
follow_until_stopped(-1)

# Finally finish the run to home
tc.SetPTZ(pan=1.57,tilt=0.2)
w4roi = tc.WaitForROI(foreground=False,wrtOrigin=True,roi_x=0.0, roi_y=0.0, roi_radius=10.0)
tc.addCondition(ConditionIsCompleted("ROI detector",tc,w4roi))
follow_until_stopped(+1)

# Finally mark completion by pointing toward the shore
tc.AlignWithShore(angle=3.14, ang_velocity=1.0)

rospy.loginfo("Mission completed")

