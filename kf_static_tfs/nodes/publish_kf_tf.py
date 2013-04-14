#!/usr/bin/env python
import roslib; roslib.load_manifest('kf_static_tfs')
import rospy
import math
from geometry_msgs.msg import Vector3Stamped,Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
import tf
from tf.transformations import quaternion_from_euler

base_name = "/kingfisher"
base_frame ="/world"
broadcaster = tf.TransformBroadcaster()
first = True
world_origin = None
marker_pub = None

def odom_cb(data):
    global broadcaster, base_frame, base_name
    global first, world_origin
    p = data.pose.pose.position
    if first:
        world_origin = p
        first = False
    q = data.pose.pose.orientation
    broadcaster.sendTransform((p.x-world_origin.x,p.y-world_origin.y,0),
            (q.x,q.y,q.z,q.w),data.header.stamp,base_name+"/odom",base_name+"/start_water")
    broadcaster.sendTransform((0,0,p.z-world_origin.z),
            (0,0,0,1),data.header.stamp,base_name+"/start_water",base_name+"/start")
    broadcaster.sendTransform((world_origin.x,world_origin.y,world_origin.z),
            (0,0,0,1),data.header.stamp,base_name+"/start",base_frame)


def imu_cb(data):
    global broadcaster, base_name
    q = quaternion_from_euler(data.vector.x,data.vector.y,data.vector.z)
    broadcaster.sendTransform((0,0,0),
            q,data.header.stamp,base_name+"/base",base_name+"/odom")


def mag_cb(data):
    global marker_pub
    x = data.vector.x/100.
    y = data.vector.y/100.
    z = data.vector.z/100.
    marker = Marker()
    marker.header = data.header
    # debug:
    # marker.header.frame_id = "/kingfisher/odom"
    marker.ns = "imu_markers"
    marker.id = 10
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.points = [Point(0,0,0),Point(x,y,z)]
    marker.scale.x = 0.5
    marker.scale.y = 0.5
    marker.scale.z = 0.0;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.lifetime.secs=-1.0
    marker_pub.publish(marker)



if __name__ == '__main__':
    try:
        rospy.init_node("axis_tf_broadcaster")
        base_frame = rospy.get_param("~base_frame",base_frame)
        base_name = rospy.get_param("~base_name",base_name)
        odom_sub = rospy.Subscriber("/gps/odom",Odometry,odom_cb)
        imu_sub = rospy.Subscriber("/imu/rpy",Vector3Stamped,imu_cb)
        mag_sub = rospy.Subscriber("/imu/mag",Vector3Stamped,mag_cb)
        marker_pub = rospy.Publisher("/imu/marker",Marker)
        rospy.loginfo("Started Odom/Imu KF TF broadcaster")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
