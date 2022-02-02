#! /usr/bin/env python3

import rospy
import math
import numpy as np
import tf2_ros
import tf_conversions


from std_msgs.msg import Float32, Float64, Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import JointState, Imu


class VESC_simInterface:
    def __init__(self):
        self.max_str_angle=rospy.get_param('max_str_angle',0.2)
        self.str_ang = 0.0
        self.spd_cmd = 0.0
        self.time_out = 0.0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.yawRate = 0.0
        self.dt = 0.05
        self.quat = (0,0,0,1)
        self.speed = 0.0


        #self.speed_pub = rospy.Publisher('speed',Float64,queue_size=5)
        self.qtm_sub = rospy.Subscriber("joint_states",JointState,self.speedCallback)
        self.imu_sub = rospy.Subscriber("imu",Imu,self.IMUCallback)
        self.ack_sub = rospy.Subscriber("ackermann_cmd", AckermannDriveStamped, self.ackerCallBack)


        self.pub_vel_left_rear_wheel = rospy.Publisher('left_rear_wheel_speed_controller/command', Float64, queue_size=1)
        self.pub_vel_right_rear_wheel = rospy.Publisher('right_rear_wheel_speed_controller/command', Float64, queue_size=1)
        self.pub_vel_left_front_wheel = rospy.Publisher('left_front_wheel_speed_controller/command', Float64, queue_size=1)
        self.pub_vel_right_front_wheel = rospy.Publisher('right_front_wheel_speed_controller/command', Float64, queue_size=1)

        self.pub_pos_left_steering_hinge = rospy.Publisher('left_steering_hinge_position_controller/command', Float64, queue_size=1)
        self.pub_pos_right_steering_hinge = rospy.Publisher('right_steering_hinge_position_controller/command', Float64, queue_size=1)
        self.odom_broadcaster = tf2_ros.TransformBroadcaster()
        self.odom_pub = rospy.Publisher('odom',Odometry,queue_size=1)

        self.send_timer = rospy.Timer(rospy.Duration(self.dt), self.set_throttle_steer_callback)
        self.odom_timer = rospy.Timer(rospy.Duration(self.dt),self.odomTFCallback)



    def speedCallback(self,msg):
        speedlf = msg.velocity[0]
        speedlr = msg.velocity[1]
        speedrf = msg.velocity[3]
        speedrr = msg.velocity[4]
        speedavg = (speedlf + speedlr + speedrf + speedrr)/4.0
        self.speed = speedavg*0.05 # v=omega*r, r=0.05 wheel radius
        #speed = Float64()
        #speed.data = speedavg*0.05 # v=omega*r, r=0.05 wheel radius

        # publish ackermann message
        # self.speed_pub.publish(speed)

    def IMUCallback(self,msg):
        self.yawRate = msg.angular_velocity.z
        oriQuat = msg.orientation
        self.quat = (oriQuat.x, oriQuat.y, oriQuat.z, oriQuat.w)
        roll, pitch, yaw = tf_conversions.transformations.euler_from_quaternion(self.quat)
        self.yaw = yaw


    def ackerCallBack(self,msg):
        self.str_ang = msg.drive.steering_angle
        self.spd_cmd = msg.drive.speed
        self.time_out = rospy.get_time()

    def odomTFCallback(self,msg):
        self.x = self.x + self.speed*math.cos(self.yaw)*self.dt
        self.y = self.y + self.speed*math.sin(self.yaw)*self.dt
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = self.quat[0]
        t.transform.rotation.y = self.quat[1]
        t.transform.rotation.z = self.quat[2]
        t.transform.rotation.w = self.quat[3]
        self.odom_broadcaster.sendTransform(t)

        Od = Odometry()
        Od.header.stamp = rospy.Time.now()
        Od.header.frame_id = "odom"
        Od.child_frame_id = "base_link"
        Od.pose.pose.position.x = self.x
        Od.pose.pose.position.y = self.y
        Od.pose.pose.position.z = 0.0
        Od.pose.pose.orientation.x = self.quat[0]
        Od.pose.pose.orientation.y = self.quat[1]
        Od.pose.pose.orientation.z = self.quat[2]
        Od.pose.pose.orientation.w = self.quat[3]
        Od.twist.twist.linear.x = self.speed
        Od.twist.twist.angular.z = self.yawRate
        self.odom_pub.publish(Od)


        print("sent odom")

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
