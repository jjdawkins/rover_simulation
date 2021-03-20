#! /usr/bin/env python3

import rospy
import math
import numpy as np


from ackermann_msgs.msg import *
from std_msgs.msg import Empty, String, Header, Float64
from sensor_msgs.msg import Joy


class roverInterfaceSim:
    def __init__(self):


        self.acker_sub = rospy.Subscriber('acker_cmd',AckermannDriveStamped,self.ackerCallBack)
        self.joy_sub = rospy.Subscriber('/joy',Joy,self.joyCallBack)
        self.left_front_thr_pub = rospy.Publisher('left_front_wheel_effort_controller/command',Float64,queue_size=10)
        self.right_front_thr_pub = rospy.Publisher('right_front_wheel_effort_controller/command',Float64,queue_size=10)
        self.left_rear_thr_pub = rospy.Publisher('left_front_wheel_effort_controller/command',Float64,queue_size=10)
        self.right_rear_thr_pub = rospy.Publisher('right_front_wheel_effort_controller/command',Float64,queue_size=10)
        self.left_str_ang_pub = rospy.Publisher('left_steering_hinge_position_controller/command',Float64,queue_size=10)
        self.right_str_ang_pub = rospy.Publisher('right_steering_hinge_position_controller/command',Float64,queue_size=10)
        self.send_timer = rospy.Timer(rospy.Duration(0.05), self.sendCallBack)
        self.str_ang = 0
        self.thr_cmd = 0
        self.time_out = rospy.get_time()
        self.auto = False


    def joyCallBack(self,msg):
        self.auto = False
        self.thr_cmd = self.max_thr_cmd*msg.axes[1]
        self.str_ang  = self.max_str_angle*msg.axes[2]

    def ackerCallBack(self,msg):
        self.auto = True
        self.str_ang = msg.drive.steering_angle
        self.thr_cmd = msg.drive.acceleration
        self.time_out = rospy.get_time();

    def sendCallBack(self,msg):
        thr_cmd = Float64()

        if(self.auto):
            rospy.loginfo("%f",(rospy.get_time()-self.time_out))
            if((rospy.get_time()-self.time_out) < 0.3):
                thr_cmd.data = self.thr_cmd
            else:
                thr_cmd.data = 0;
        else:
            thr_cmd.data = self.thr_cmd

        self.left_front_thr_pub.publish(thr_cmd)
        self.right_front_thr_pub.publish(thr_cmd)
        self.left_rear_thr_pub.publish(thr_cmd)
        self.right_rear_thr_pub.publish(thr_cmd)
        str_cmd = Float64()

        if(self.str_ang > self.max_str_angle):
            self.str_ang = self.max_str_angle

        if(self.str_ang < -self.max_str_angle):
            self.str_ang = -self.max_str_angle

        str_cmd.data = self.str_ang
        self.left_str_ang_pub.publish(str_cmd)
        self.right_str_ang_pub.publish(str_cmd)



if __name__ == '__main__':
    rospy.init_node('Rover_Sim_Interface_Node')

    myRover = roverInterfaceSim()

    rospy.spin()
