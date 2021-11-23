#! /usr/bin/env python

import rospy
from std_msgs.msg import Float32
from mDev import *
from geometry_msgs.msg import Twist
import time
from echopkg.srv import tom_srv, tom_srvRequest


mdev = mDEV()
var = Twist()
rospy.init_node('move_node_wall')


def callback(msg):
   
  if msg.data > 30:
    var.linear.x = .5
    var.angular.z = 0

  if msg.data < 20:
    var.linear.x = -.5
    var.angular.z = -.5
  
  pub.publish(var)

def callback_stop(msg):
  var.linear.x = 0
  var.angular.z = 0
  pub.publish(var)


#kaller service, venter paa svar

rospy.wait_for_service('crash_service')

crash_detect_service = rospy.ServiceProxy('crash_service', tom_srv)
crash_detect_object = tom_srvRequest()
crash_detect_object.request = 'klart?'


result = crash_detect_service(crash_detect_object)
print(result)


#faatt svar, fortsetter program

if result.response == "Det er klart":
  pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
  sub = rospy.Subscriber('/echo', Float32, callback, queue_size = 1)
  
  

elif result.response == "Ikke klart":
  pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
  sub = rospy.Subscriber('/echo', Float32, callback_stop, queue_size = 1)
  
  
  
rospy.spin()