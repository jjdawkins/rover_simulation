#! /usr/bin/env python3

import rospy
import math
import numpy as np


from std_msgs.msg import Float32, Float64, Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import JointState


class VESC_simInterface:
    def __init__(self):
        self.speed_pub = rospy.Publisher('speed',Float64,queue_size=5)
        self.max_str_angle=rospy.get_param('max_str_angle',0.2)
        self.qtm_sub = rospy.Subscriber("joint_states",JointState,self.speedCallback)
        self.ack_sub = rospy.Subscriber("ackermann_cmd", AckermannDriveStamped, self.ackerCallBack)


        self.pub_vel_left_rear_wheel = rospy.Publisher('left_rear_wheel_speed_controller/command', Float64, queue_size=1)
        self.pub_vel_right_rear_wheel = rospy.Publisher('right_rear_wheel_speed_controller/command', Float64, queue_size=1)
        self.pub_vel_left_front_wheel = rospy.Publisher('left_front_wheel_speed_controller/command', Float64, queue_size=1)
        self.pub_vel_right_front_wheel = rospy.Publisher('right_front_wheel_speed_controller/command', Float64, queue_size=1)

        self.pub_pos_left_steering_hinge = rospy.Publisher('left_steering_hinge_position_controller/command', Float64, queue_size=1)
        self.pub_pos_right_steering_hinge = rospy.Publisher('right_steering_hinge_position_controller/command', Float64, queue_size=1)
        self.send_timer = rospy.Timer(rospy.Duration(0.05), self.set_throttle_steer_callback)

        self.str_ang = 0.0
        self.spd_cmd = 0.0
        self.time_out = 0.0

    def speedCallback(self,msg):
        speedlf = msg.velocity[0]
        speedlr = msg.velocity[1]
        speedrf = msg.velocity[3]
        speedrr = msg.velocity[4]
        speedavg = ( abs(speedlf) + abs(speedlr) + abs(speedrf) + abs(speedrr))/4.0
        speed = Float64()
        speed.data = speedavg*0.05 # v=omega*r, r=0.05 wheel radius

        # publish ackermann message
        self.speed_pub.publish(speed)

    def ackerCallBack(self,msg):
        self.str_ang = msg.drive.steering_angle
        self.spd_cmd = msg.drive.speed
        self.time_out = rospy.get_time();

    def set_throttle_steer_callback(self,msg):

        rospy.loginfo("%f",(rospy.get_time()-self.time_out))
        if((rospy.get_time()-self.time_out) < 0.3):
            ang_spd = self.spd_cmd/0.05 # 0.05 is wheel radius
            steer = self.str_ang
        else:
            ang_spd = 0.0 # 0.05 is wheel radius
            steer = 0.0

        self.pub_vel_left_rear_wheel.publish(ang_spd)
        self.pub_vel_right_rear_wheel.publish(ang_spd)
        self.pub_vel_left_front_wheel.publish(ang_spd)
        self.pub_vel_right_front_wheel.publish(ang_spd)


        if(steer > self.max_str_angle):
            steer = self.max_str_angle
        if(steer < -self.max_str_angle):
            steer = -self.max_str_angle

        self.pub_pos_left_steering_hinge.publish(steer)
        self.pub_pos_right_steering_hinge.publish(steer)


if __name__ == '__main__':
    rospy.init_node('VESC_servo_interface')

    myCar = VESC_simInterface()

    rospy.spin()
