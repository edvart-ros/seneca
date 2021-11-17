#! /usr/bin/env python

import cv2
import numpy as np
import time
from mDev import *
import rospy
from std_msgs.msg import String, Float32
from echopkg.msg import color_msg



cam = cv2.VideoCapture(0)  # velger kameraet

rospy.init_node('center_color_node')
pub = rospy.Publisher('center_color', color_msg, queue_size = 1) #publiserer til ny topic med egen melding
msg = color_msg()


while not rospy.is_shutdown():   #loop

  
  img = cam.read()[1]
  width = int(img.shape[1] * 30 /100)
  height = int(img.shape[0] * 30 /100)
  dim = (width, height)
  img = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
  cv2.imshow('resized image', img)
  color = img[100, 100]

  
  msg.Blue.data = color[1]
  msg.Green.data = color[2]
  msg.Red.data = color[0]
  
  pub.publish(msg)
  
  if cv2.waitKey(1) == ord("q"):    #avslutter loopen og slutter a publisere ved tastetrykk q
    break
  
    
cam.release()             #"releaser" kamerate slik at andre programmer kan benytte det
cv2.destroyAllWindows()