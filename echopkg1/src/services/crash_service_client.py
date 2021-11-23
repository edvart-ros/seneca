#! /usr/bin/env python

import rospy
from echopkg.srv import tom_srv, tom_srvRequest
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist


    
rospy.init_node('crash_service_client')

rospy.wait_for_service('crash_service')

crash_detect_service = rospy.ServiceProxy('crash_service', tom_srv)
crash_detect_object = tom_srvRequest()

result = crash_detect_service()
print(result)


pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
var = Twist()




while True:
  var.linear.x = 0.5
  pub.publish(var)
