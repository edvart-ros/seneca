#! /usr/bin/env python

import cv2
import numpy as np
import time
from geometry_msgs.msg import Twist
import rospy
from std_msgs.msg import String, Float32
from echopkg.msg import color_msg

rospy.init_node('line_follower')

pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
cam = cv2.VideoCapture(0)  # velger kameraet

var = Twist()
var.linear.x = 0.4
pub.publish(var)
time.sleep(0.5)
var.linear.x = .30
pub.publish(var)
time.sleep(1)
var.linear.x = .25
pub.publish(var)


img = cam.read()[1]  
width = int(img.shape[1] * 15 /100)
height = int(img.shape[0] * 15 /100)
dim = (width, height)


height, width, channels = img.shape

lower_yellow = np.array([20, 100, 100])
upper_yellow = np.array([50, 255, 255])

prev = 0


while True:

  e1 = cv2.getTickCount()

  img = cam.read()[1]
  img = cv2.resize(img, dim, interpolation = cv2.INTER_NEAREST)
  hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV).astype(np.float)
  mask = cv2.inRange(hsv_img, lower_yellow, upper_yellow)
  res = cv2.bitwise_and(img, img, mask = mask)

  m = cv2.moments(mask, False)

  try:
    cx, cy = m['m10']/m['m00'], m['m01']/m['m00']

  except ZeroDivisionError:
    cy, cx = height/2, width/2


  cv2.circle(res, (int(cx), int(cy)), 5, (0,0,255), -1)


  cv2.imshow('RES', res)
  #cv2.imshow('img', img)

  error_x = cx - width / 2;

  var.angular.z = round(    ((((-error_x-240)/75)/1.7-0.25)*2)/1.5     , 2)
  var.linear.x = 0.25

  new = var.angular.z


  if var.angular.z < -0.89:
    var.angular.z = 0
    #var.linear.x = 0



  if (new - prev > 0.05) or (new-prev < -0.05):
    #print('change = ', (new-prev))
    prev = var.angular.z 
    pub.publish(var)
    print(var)

    #lagrer sammenligninsverdi





  #print('hastighet', var.linear.x,'    styring', var.angular.z)
  if cv2.waitKey(1) == ord("q"):    #avslutter loopen og slutter a publisere ved tastetrykk q
    break


i = 0

while i < 10:
  var.linear.x = 0
  var.angular.z = 0
  pub.publish(var)
  i += 1

cam.release()             #"releaser" kamerate slik at andre programmer kan benytte det
cv2.destroyAllWindows()
