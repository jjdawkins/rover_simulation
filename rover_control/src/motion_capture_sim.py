#! /usr/bin/env python3

import rospy
import math
import numpy as np
import tf2_ros
import tf_conversions


from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped, Quaternion, Pose
from ackermann_msgs.msg import AckermannDriveStamped
from gazebo_msgs.msg import ModelStates


class motionCapture_simInterface:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.dt = 0.04
        self.quat = Quaternion()
        self.vels = Twist()


        self.qtm_sub = rospy.Subscriber("/gazebo/model_states",ModelStates,self.GazeboCallback)

        self.odom_broadcaster = tf2_ros.TransformBroadcaster()
        self.odom_pub = rospy.Publisher('mocap/odom',Odometry,queue_size=1)
        self.odom_timer = rospy.Timer(rospy.Duration(self.dt),self.odomTFCallback)


    def GazeboCallback(self,msg):
        ind = msg.name.index('rover')
        #print(msg.name[ind])
        #print(msg.pose[ind])
        self.x = msg.pose[ind].position.x
        self.y = msg.pose[ind].position.y
        self.z = msg.pose[ind].position.z
        self.quat = msg.pose[ind].orientation
        self.vels = msg.twist[ind]


    def odomTFCallback(self,msg):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "mocap"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = self.z
        t.transform.rotation.x = self.quat.x
        t.transform.rotation.y = self.quat.y
        t.transform.rotation.z = self.quat.z
        t.transform.rotation.w = self.quat.w
        self.odom_broadcaster.sendTransform(t)

        Od = Odometry()
        Od.header.stamp = rospy.Time.now()
        Od.header.frame_id = "mocap"
        Od.child_frame_id = "base_link"
        Od.pose.pose.position.x = self.x
        Od.pose.pose.position.y = self.y
        Od.pose.pose.position.z = self.z
        Od.pose.pose.orientation.x = self.quat.x
        Od.pose.pose.orientation.y = self.quat.y
        Od.pose.pose.orientation.z = self.quat.z
        Od.pose.pose.orientation.w = self.quat.w
        Od.twist.twist = self.vels
        self.odom_pub.publish(Od)

if __name__ == '__main__':
    rospy.init_node('VESC_servo_interface')

    myCar = motionCapture_simInterface()

    rospy.spin()
