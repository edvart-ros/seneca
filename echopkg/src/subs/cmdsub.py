#! /usr/bin/env python

import rospy
from mDev import *
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from mDev import *


def callback(var):

    pwm = var.linear.x * 1000
    
    if pwm > 1000 or pwm < -1000:
      rospy.loginfo('linear.x out of range!')
    
    else:
      angle = (var.angular.z+1) *90
      if angle < 10:
        angle = 10
      elif angle > 170:
        angle = 170
      else: angle = (var.angular.z+1) * 90
        
      mdev.move(pwm, pwm, angle)
      print('mdev.move', pwm, pwm, angle)
      

rospy.init_node('bil_topic')
print('startet node')
sub = rospy.Subscriber('/cmd_vel', Twist, callback, queue_size = 1)      
pub = rospy.Publisher('/echo', Float32, queue_size = 1)
rate = rospy.Rate(15)
var = Twist()
mdev = mDEV()


echo = mdev.getSonic()
pub.publish(echo)




while not rospy.is_shutdown():
  echo = mdev.getSonic()
  pub.publish(echo)
  rate.sleep()

rospy.spin()






 

