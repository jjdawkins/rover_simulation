#!/usr/bin/env python3
import rospy

# Because of transformations
import tf_conversions

import tf2_ros
from geometry_msgs.msg import TransformStamped, Pose, Quaternion
from nav_msgs.msg import Odometry


def handle_rover_odom(msg):
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "map"
    t.child_frame_id = "base_link"
    t.transform.translation.x = msg.pose.pose.position.x
    t.transform.translation.y = msg.pose.pose.position.y
    t.transform.translation.z = msg.pose.pose.position.z
    t.transform.rotation = msg.pose.pose.orientation
    #t.transform.rotation.x = msg.pose.pose.orientation.x
    #t.transform.rotation.y = msg.pose.pose.orientation.y
    #t.transform.rotation.z = msg.pose.pose.orientation.z
    #t.transform.rotation.w = msg.pose.pose.orientation.x

    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('tf2_rover_broadcaster')
    rospy.Subscriber('rover/odom',Odometry, handle_rover_odom)
    rospy.spin()
