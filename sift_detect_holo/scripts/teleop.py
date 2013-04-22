#!/usr/bin/env python
import roslib; roslib.load_manifest('sift_detect_holo')
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

last_joy = -1e10
joy_value = None

def joy_cb(value):
    global joy_value
    global last_joy
    last_joy = rospy.rostime.get_time()
    joy_value = value

def talker():
    global joy_value
    global last_joy
    rospy.init_node('vrep_ros_teleop')
    sub = rospy.Subscriber('~joy', Joy, joy_cb)
    pub = rospy.Publisher('~twistCommand', Twist)
    camera_pubf = rospy.Publisher('~panCommand_float', Float64)
    camera_pubt = rospy.Publisher('~panCommand_twist', Twist)
    axis_linear = rospy.get_param("~axis_linear",1)
    axis_angular = rospy.get_param("~axis_angular",0)
    axis_angular_pan = rospy.get_param("~axis_angular_pan",3)
    scale_linear = rospy.get_param("~scale_linear",5.)
    scale_angular = rospy.get_param("~scale_angular",10.)
    scale_angular_pan = rospy.get_param("~scale_angular_pan",10.)
    timeout = True
    while not rospy.is_shutdown():
        twist = Twist()
        pan_vel = Float64()
        pan_twist = Twist()
        if (rospy.rostime.get_time() - last_joy) < 0.5: 
            if timeout:
                timeout = False
                rospy.loginfo("Accepting joystick commands")
            twist.linear.x = joy_value.axes[axis_linear] * scale_linear
            twist.angular.z = joy_value.axes[axis_angular] * scale_angular
            pan_vel.data = joy_value.axes[axis_angular_pan] * scale_angular_pan
            pan_twist.angular.z = joy_value.axes[axis_angular_pan] * scale_angular_pan
            
            if twist.linear.x < 0:
                twist.angular.z = - twist.angular.z
        else:
            if not timeout:
                timeout = True
                rospy.loginfo("Timeout: ignoring joystick commands")
        pub.publish(twist)
        camera_pubf.publish(pan_vel)
        camera_pubt.publish(pan_twist)
        rospy.sleep(0.1)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
