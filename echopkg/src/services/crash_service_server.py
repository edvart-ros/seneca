#! /usr/bin/env python

import rospy
from echopkg.srv import tom_srv, tom_srvResponse, tom_srvRequest
from echopkg.msg import color_msg
from std_msgs.msg import Float32


  
class Robot(object):
  
  def __init__(self):
    self.crash_service = rospy.Service('/crash_service', tom_srv, self.service_callback)
    self.sub = rospy.Subscriber('echo', Float32, self.sub_cb, queue_size = 1)
    self.rospy = rospy
    self.avstand = Float32
    self.response = tom_srvResponse()
    
  def sub_cb(self, msg):
    self.avstand = msg.data
    return self.avstand
    
  def service_callback(self, request):
  
    rospy.loginfo('request received')
  
    if request.request == "klart?":
      if self.avstand > 20:
        self.response = "Det er klart"
      else:
        self.response = "Ikke klart"
    else:
      rospy.loginfo('invalid request')
      self.response = "ugyldig request"
         
    
    return self.response


if __name__ == '__main__':
  print('initializing node')
  rospy.init_node('crash_service')
  Robot()
  print('service started')
  rospy.spin()

