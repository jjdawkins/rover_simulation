#! /usr/bin/env python3

import rospy
import math
import numpy as np

from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import tf
#from tf_conversions.transformations import quaternion_to_euler


class speed_publisher:
    def __init__(self):
        self.speed_pub = rospy.Publisher('speed',Float64,queue_size=5)
        self.qtm_sub = rospy.Subscriber('odom',Odometry,self.odomCallback)

    def odomCallback(self,msg):
        speedx = msg.twist.twist.linear.x
        speedy = msg.twist.twist.linear.y
        speed = Float64()
        speed.data = math.sqrt(speedx*speedx + speedy*speedy)

        # publish ackermann message
        self.speed_pub.publish(speed)

if __name__ == '__main__':
    rospy.init_node('speed_publisher')

    mySPD = speed_publisher()

    rospy.spin()
