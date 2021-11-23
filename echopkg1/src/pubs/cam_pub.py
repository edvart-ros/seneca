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
mdev = mDEV()

img = cam.read()[1]  
width = int(img.shape[1] * 25 /100)
height = int(img.shape[0] * 25 /100)
dim = (width, height)
img = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
print(img.shape)
shape = img.shape

mdev.setLed(1, 1, 1)

while not rospy.is_shutdown():   #loop

  
  img = cam.read()[1]
  img = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
  cv2.imshow('resized image', img)
  color = img[(shape[0]/2), (shape[1]/2)]

  
  msg.Green.data = float(color[1])/255
  msg.Red.data = float(color[2])/255
  msg.Blue.data = float(color[0])/255
  
  
  
  pub.publish(msg)
  
  if cv2.waitKey(1) == ord("q"):    #avslutter loopen og slutter a publisere ved tastetrykk q
    break
    
  

  
    
cam.release()             #"releaser" kamerate slik at andre programmer kan benytte det
cv2.destroyAllWindows()