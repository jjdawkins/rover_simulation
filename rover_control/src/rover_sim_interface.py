#! /usr/bin/env python3

import rospy
import math
import numpy as np

from ackermann_msgs.msg import *
from std_msgs.msg import Empty, String, Header, Float64


class roverInterfaceSim:
    def __init__(self):

        self.max_str_angle = rospy.get_param('max_str_angle',0.75)
        self.max_speed = rospy.get_param('max_speed',2)

        self.acker_sub = rospy.Subscriber('acker_cmd',AckermannDriveStamped,self.ackerCallBack)
        self.left_front_spd_pub = rospy.Publisher('left_front_wheel_velocity_controller/command',Float64,queue_size=10)
        self.right_front_spd_pub = rospy.Publisher('right_front_wheel_velocity_controller/command',Float64,queue_size=10)
        self.left_rear_spd_pub = rospy.Publisher('left_front_wheel_velocity_controller/command',Float64,queue_size=10)
        self.right_rear_spd_pub = rospy.Publisher('right_front_wheel_velocity_controller/command',Float64,queue_size=10)
        self.left_str_ang_pub = rospy.Publisher('left_steering_hinge_position_controller/command',Float64,queue_size=10)
        self.right_str_ang_pub = rospy.Publisher('right_steering_hinge_position_controller/command',Float64,queue_size=10)
        self.send_timer = rospy.Timer(rospy.Duration(0.1), self.sendCallBack)
        self.str_ang = 0
        self.thr_cmd = 0


    def ackerCallBack(self,msg):
        self.str_ang = msg.drive.steering_angle
        self.thr_cmd = msg.drive.acceleration

    def sendCallBack(self,msg):
        spd_cmd = Float64()
        spd_cmd.data = self.thr_cmd

        self.left_front_spd_pub.publish(spd_cmd)
        self.right_front_spd_pub.publish(spd_cmd)
        self.left_rear_spd_pub.publish(spd_cmd)
        self.right_rear_spd_pub.publish(spd_cmd)

        str_cmd = Float64()
        str_cmd.data = self.str_ang
        self.left_str_ang_pub.publish(str_cmd)
        self.right_str_ang_pub.publish(str_cmd)









if __name__ == '__main__':
    rospy.init_node('Goats_State_Sender')

    myRover = roverInterfaceSim()

    rospy.spin()
