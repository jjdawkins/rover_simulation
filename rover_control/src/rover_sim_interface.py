#! /usr/bin/env python3

import rospy
import time
import math
import numpy as np


from ackermann_msgs.msg import *
from std_msgs.msg import Empty, String, Header, Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from rover_control.msg import status
from rover_control.srv import setArm, setMode

red = (255,0,0)
white =(255,255,255)
yellow = (255,255,0)
green = (0,255,0)
blue = (0,0,255)
black = (0,0,0)


class RoverSimInterface():

    def __init__(self):

        self.max_str_angle = rospy.get_param('~max_str_angle',0.5)
        self.encoder_constant = rospy.get_param('~encoder_constant',3000)
        self.wheel_base = rospy.get_param('~wheel_base',0.4)
        self.time_out = 0
        self.auto_mode = False
        self.armed = False
        self.str_cmd = 0
        self.thr_cmd = 0
        self.spd_cmd = 0
#        self.led_strip = dotstar.DotStar(board.D17, board.D18, 10, brightness=1.0)
        self.led_color = red
        self.ctrl_dt = 0.05

        self.max_speed = rospy.get_param('~max_speed',2)
        self.spd_cmd_pub = rospy.Publisher('/rover/commands/motor/speed',Float64,queue_size=10)
        self.str_cmd_pub = rospy.Publisher('/rover/commands/servo/position',Float64,queue_size=10)
        self.status_pub = rospy.Publisher('status',status,queue_size=2)
        self.acker_sub = rospy.Subscriber('acker_cmd',AckermannDriveStamped,self.ackerCallBack)
        self.cmd_sub = rospy.Subscriber('cmd_vel',Twist,self.cmdVelCallBack)
        self.arm_srv = rospy.Service('arm',setArm,self.armCallBack)
        self.mode_srv = rospy.Service('mode',setMode,self.modeCallBack)


        self.joy_sub = rospy.Subscriber('/joy',Joy,self.joyCallBack)

        self.status_timer = rospy.Timer(rospy.Duration(0.3), self.statusUpdate)
        self.ctlr_timer = rospy.Timer(rospy.Duration(self.ctrl_dt), self.controlLoop)

    def __del__(self):
        print('Destructor called, Shutdown Initiated')
        #self.led_strip.fill(black)

    def armCallBack(self,req):
        self.armed = req.arm
        rospy.loginfo("Armed %d",self.armed)
        return self.armed

    def modeCallBack(self,req):
        self.auto_mode = req.mode
        return self.auto_mode


    def ackerCallBack(self,msg):
        #rospy.loginfo("acker_Callback")
        if(self.auto_mode):
            self.str_cmd = msg.drive.steering_angle/self.max_str_angle

            self.spd_cmd = msg.drive.speed
            #rospy.loginfo("Str_PWM: %f, Thr_PWM: %f",str_cmd,thr_cmd)

        self.time_out = rospy.get_time();

    def cmdVelCallBack(self,msg):
        rospy.loginfo("cmdvel_Callback")

        if(abs(msg.linear.x) > 0.1):
            self.str_cmd = math.atan(msg.angular.z*self.wheel_base/msg.linear.x)/self.max_str_angle
        else:
            self.str_cmd = math.abs(msg.linear.x)*msg.angular.z/self.max_str_angle

        self.spd_cmd = msg.linear.x
            #rospy.loginfo("Str_PWM: %f, Thr_PWM: %f",str_cmd,thr_cmd)

        self.time_out = rospy.get_time();


    # defines behavior in response to user joystick inputs
    def joyCallBack(self,msg):
        #rospy.loginfo("Joy_Callback")

        if(msg.buttons[0]): # A button, arm/disarm
            self.armed = not self.armed
            rospy.loginfo("Armed %d",self.armed)

        if(msg.buttons[2]): # X button, disable auto mode
            self.auto_mode = False
            rospy.loginfo("Auto Mode Disable")


        if(msg.buttons[3]): # Y button on game pad, enable auto mode
            self.auto_mode = True
            rospy.loginfo("Auto Mode Enable")

        # forward speed and steering commands from left (speed) and right (steering) sticks
        self.spd_cmd = self.max_speed*msg.axes[1]
        self.str_cmd = msg.axes[3]

    # publishes the vehicle status (arm status and mode status) to status topic
    def statusUpdate(self,event):
        stat_msg=status()

        stat_msg.armed = self.armed
        if(self.armed):
            self.led_color = green

        else:
            self.led_color = red

        if(self.auto_mode):
            stat_msg.mode = 1
#            self.led_strip.fill(self.led_color)
#            rospy.sleep(0.05)
#            self.led_strip.fill(black)
#            rospy.sleep(0.05)
#            self.led_strip.fill(self.led_color)
#            rospy.sleep(0.05)
#            self.led_strip.fill(black)
#            rospy.sleep(0.05)
#            self.led_strip.fill(self.led_color)
        else:
            stat_msg.mode = 0
#            self.led_strip.fill(self.led_color)

        self.status_pub.publish(stat_msg)

    def controlLoop(self,event):

        if(rospy.is_shutdown()):
#            self.led_strip.fill(black)
            rospy.sleep(0.05)

        thr_cmd = 0
        spd_cmd = 0
        str_cmd = 0

        if(self.auto_mode):
            if((rospy.get_time()-self.time_out) < 0.3):
                spd_cmd = self.spd_cmd
            else:
                spd_cmd = 0
        else:
            spd_cmd = self.spd_cmd

        str_cmd = self.str_cmd*0.5 + 0.5
        str_cmd_msg = Float64()
        str_cmd_msg.data = str_cmd
        self.str_cmd_pub.publish(str_cmd_msg)

        if(self.armed):
            mot_cmd_msg = Float64()
            mot_cmd_msg.data = 3092.53*self.spd_cmd
            # Change here to publish to VESC control topic (PWM or speed if PWM is not compatible)
            #print(mot_cmd_msg)
            self.spd_cmd_pub.publish(mot_cmd_msg)
            ##self.thr_pwm.set_duty_cycle(thr_cmd)
            # ADD CODE HERE TO PUBLISH TO VESC SPEED CONTROL TOPIC
        else:
            mot_cmd_msg = Float64()
            self.spd_cmd_pub.publish(mot_cmd_msg)
            # change here to publish a speed of zero to the vesc speed topic
            #self.thr_pwm.set_duty_cycle(1.5)



if __name__ == '__main__':

    try:
        rospy.init_node('Rover_Interface_Node')
        my_rover = RoverSimInterface()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()
