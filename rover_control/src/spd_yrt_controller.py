#! /usr/bin/env python3

import rospy
import math
import numpy as np

from ackermann_msgs.msg import *
from std_msgs.msg import Empty, String, Header, Float64
from nav_msgs.msg import Odometry

class roverControl:
    def __init__(self):

        self.max_str_angle = rospy.get_param('max_str_angle',0.75)
        self.max_speed = rospy.get_param('max_speed',2)
        self.L = rospy.get_param('wheel_base',0.2) # 0.2 meters
        self.KpSpd = rospy.get_param('Kp_spd',0.1)
        self.KiSpd = rospy.get_param('Ki_spd',0.02)

        self.acker_pub = rospy.Publisher('acker_cmd',AckermannDriveStamped,self.ackerCallBack)
        self.pose_sub = rospy.Subscriber('cmd_vel',Twist,self.twistCallback)
        self.qtm_sub = rospy.Subscriber('/qualisys/Truck1/odom',Odometry,self.odomCallback)
        self.send_timer = rospy.Timer(rospy.Duration(0.1), self.sendCallBack)

        self.ackMsg = AckermannDriveStamped()
        self.twistMsg = Twist()
        self.yawRate = 0.0
        self.desSpeed = 0.0

    def twistCallback(self,msg):
        self.yawRate = msg.angular.z
        self.desSpeed = msg.linear.x

    def odomCallback(self,msg):
        speedx = msg.twist.linear.x
        speedy = msg.twist.linear.y
        self.speed = math.sqrt(speedx^2 + speedy^2)

    def sendCallBack(self,msg):
        # Steering controller (using yaw rate speed kinematics)
        steerAng = Float64()
        if self.speed<0.1: # if speed less than 10 cm/s, use speed invariant turn angle
            steerAng = math.atan(self.yawRate*self.L/0.1)
        else: # if speed greater than 10 cm/s, use speed dependent steering angle
            steerAng = math.atan(self.yawRate*self.L/self.speed)

        # speed controller (Using PI controller on acceleration)
        self.spdErr = self.desSpeed - self.speed

        # PWM duty cycle for throttle control
        self.thr_cmd = self.KpSpd*self.spdErr + self.KiSpd*self.spdErrInt # PI controller
        # Saturate speed command at 20% duty cycle
        if self.thr_cmd > 0.2:
            self.thr_cmd = 0.2
        elif self.thr_cmd < -0.2:
            self.thr_cmd = -0.2

        # update speed error integral term
        self.spdErrInt = self.spdErrInt + self.spdErr*0.1 # 10 Hz sampling rate
        # saturate speed error integral term at 2
        if self.spdErrInt > 2:
            self.spdErrInt = 2.0

        # package ackermann message
        self.ackMsg.steering_angle = steerAng
        self.ackMsg.acceleration = self.thr_cmd
        # publish ackermann message
        self.acker_pub.publish(self.ackMsg)



if __name__ == '__main__':
    rospy.init_node('tracking_controller')

    myRover = roverControl()

    rospy.spin()
