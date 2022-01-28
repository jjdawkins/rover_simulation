#! /usr/bin/env python3



# Edited from MIT racecar project https://github.com/mit-racecar/racecar_gazebo/blob/master/racecar_control/scripts/servo_commands.py
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDriveStamped

def set_throttle_steer(data):

    pub_vel_left_rear_wheel = rospy.Publisher('left_rear_wheel_speed_controller/command', Float64, queue_size=1)
    pub_vel_right_rear_wheel = rospy.Publisher('right_rear_wheel_speed_controller/command', Float64, queue_size=1)
    pub_vel_left_front_wheel = rospy.Publisher('left_front_wheel_speed_controller/command', Float64, queue_size=1)
    pub_vel_right_front_wheel = rospy.Publisher('right_front_wheel_speed_controller/command', Float64, queue_size=1)

    pub_pos_left_steering_hinge = rospy.Publisher('left_steering_hinge_position_controller/command', Float64, queue_size=1)
    pub_pos_right_steering_hinge = rospy.Publisher('right_steering_hinge_position_controller/command', Float64, queue_size=1)

    throttle = data.drive.speed/0.05
    steer = data.drive.steering_angle

    pub_vel_left_rear_wheel.publish(throttle)
    pub_vel_right_rear_wheel.publish(throttle)
    pub_vel_left_front_wheel.publish(throttle)
    pub_vel_right_front_wheel.publish(throttle)

    pub_pos_left_steering_hinge.publish(steer)
    pub_pos_right_steering_hinge.publish(steer)

def servo_commands():

    rospy.init_node('servo_commands', anonymous=True)

    rospy.Subscriber("ackermann_cmd", AckermannDriveStamped, set_throttle_steer)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        servo_commands()
    except rospy.ROSInterruptException:
        pass
