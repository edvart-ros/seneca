
#!/usr/bin/env python2

import cv2
import numpy as np
import time
from mDev import *

sus_array = ([500, 600, 670, 700, 670, 600, 500, 0])

cam = cv2.VideoCapture(0)
mdev = mDEV()

def sound(a):
  mdev.setBuzzer(a)
  time.sleep(0.2)

while True:

  
  img = cam.read()[1]
  width = int(img.shape[1] * 30 /100)
  height = int(img.shape[0] * 30 /100)
  dim = (width, height)
  
  img = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
  

  
  
  cv2.imshow('resized image', img)
  
  color = img[100, 100]
  
  green_value = color[1]
  red_value = color[2]
  blue_value = color[0]
  
  
  if green_value > 220:
    print('^_^')
    mdev.setLed(0, 1, 0)
    
      
    
  elif red_value > 220:
  
    print('sus!!!')
    mdev.setLed(1, 0, 0)
    for baka in sus_array:
      sound(baka)
      
      
  else:
    print('??')
    mdev.setLed(0, 0, 1)
    
  
  if cv2.waitKey(1) == ord("q"):
    break
  
    
cap.release()
cv2.destroyAllWindows()