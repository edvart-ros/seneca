#! /usr/bin/env python

import rospy
from mDev import *
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32



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
      

rospy.init_node('echo_cmd')    #node for echo publisher og cmd subscriber


sub = rospy.Subscriber('/cmd_vel', Twist, callback, queue_size = 1)      #lager topics/sub/pubs
pub = rospy.Publisher('/echo', Float32, queue_size = 1)


rate = rospy.Rate(15)   #setter rate til 15 ganger i sekundet
var = Twist()
mdev = mDEV()


echoprev = mdev.getSonic()
pub.publish(echoprev)




while not rospy.is_shutdown():    #publiserer echo saa lenge programmet gaar
  echonew = mdev.getSonic()
  
  
  if echonew == 0.00 and (echonew - echoprev) < (-5):
      pass
  elif (echonew - echoprev) > (-5):
      pub.publish(echonew)
  #print(echonew)

  echoprev = echonew
  rate.sleep()                  #sorger for at callback blir kjort kontinuerlig








 

