#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Twist


rospy.init_node("velocity_speed")
vel_pub = rospy.Publisher('omni_vel', Twist, queue_size=10)
vel = Twist()
rate = rospy.Rate(20)

vel.linear.x = 2.0     #(left wheel speed)
vel.linear.y = 2.0     #(right wheel speed)
vel.linear.z = 0

while not rospy.is_shutdown():
    vel_pub.publish(vel)
    rate.sleep()