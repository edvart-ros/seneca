#! /usr/bin/env python

import rospy
from std_msgs.msg import Float32
from mDev import *

rospy.init_node('topic_node')
pub = rospy.Publisher('/echo', Float32, queue_size = 1)
mdev = mDEV()
rate = rospy.Rate(15)

echo = mdev.getSonic()
pub.publish(echo)


while not rospy.is_shutdown():
  echo = mdev.getSonic()
  pub.publish(echo)
  rate.sleep()
  
