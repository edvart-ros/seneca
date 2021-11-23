#! /usr/bin/env python

import rospy 
from mDev import *
from geometry_msgs.msg import Twist

rospy.init_node('stop')
pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
var = Twist()
var.linear.x = 0
var.angular.z = 0

while not rospy.is_shutdown():
  pub.publish(var)