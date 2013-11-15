#!/usr/bin/python
import roslib; roslib.load_manifest('kf_launch')
import rospy
from std_msgs.msg import Header


if __name__ == '__main__':
    try:
        rospy.init_node("beat")
        rate = rospy.get_param("~rate",1.0)
        pub = rospy.Publisher("~beat",Header)
        rospy.loginfo("Started Beating at %fHz"%rate)
        pub_rate = rospy.Rate(rate);
        H = Header()
        while not rospy.is_shutdown():
            H.seq += 1
            H.stamp = rospy.Time.now()
            pub.publish(H)
            pub_rate.sleep()

    except rospy.ROSInterruptException:
        pass

