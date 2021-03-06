#! /usr/bin/env python

import rospy
from echopkg.msg import FigurAction, FigurFeedback, FigurResult
import actionlib
import time
import numpy as np
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
import cv2


#jeg vil lage en server som tar en string (figur), og en int (repetisjoner) inn som "goal".
#action server skal gi navaerende repetisjon som feedback
#action server skal returnere tid det tok a oppna malet som resultat.

class Bil(object):

  _feedback = FigurFeedback()
  
  def __init__(self):

    self._as = actionlib.SimpleActionServer('/figur_action', FigurAction, self.execute_callback, auto_start = False) #starter action server
    self.sub = rospy.Subscriber('/echo', Float32, self.sub_cb, queue_size = 1)                                    #subscriber til echo
    self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)                                                 #lager publisher til cmd_vel
    
    self._feedback = FigurFeedback()                                                                              #definerer feedback-object
    self._result = FigurResult()                                                                                  #definerer resultat-object
    self._as.start()                                                                                              #starter server
    
    self.ctrl_c = False
    self.rospy = rospy
    self.avstand = Float32()
    self.var = Twist()
    self.success = False
    self.cancelled = 0


    #linje init
    
    self.cam = cv2.VideoCapture(0)
    self.img = self.cam.read()[1]
    self.dim = int(self.img.shape[1] * 15 /100), int(self.img.shape[0] * 15 /100)
    self.height, self.width, self.channels = self.img.shape

    self.lower_yellow = np.array([20, 100, 100])
    self.upper_yellow = np.array([50, 255, 255])
    self.prev = 0
    
    self.tik = time.time()
    self.tok = time.time()
    
    
    
#############################  HJELPEFUNKSJONER, TOPIC CALLBACKS
#############################

  def sub_cb(self, msg):
    dist = msg.data
    if dist < 15 and self.cancelled == 0:
      self.rospy.loginfo("action aborted, waiting for new goal")
      self._result.tid = self.tok-self.tik
      self._as.set_aborted(self._result)
      self.success = False
      self.stopp()
      self.cancelled = 1
      return False
      
    
  def stopp(self):
    self.var.linear.x = 0
    self.var.angular.z = 0
    self.pub.publish(self.var)
    
    
    
  def firkant(self):
    self.stopp()
    time.sleep(0.1)
    
    for i in range(4):
      if self.cancelled == 1:
        break
      self.var.linear.x = 0.5
      self.pub.publish(self.var)
      time.sleep(.7)     
      if self.cancelled == 1:
        break
      self.var.angular.z = 0.7
      self.pub.publish(self.var)
      time.sleep(1.1)
      if self.cancelled == 1:
        break
      self.var.angular.z = 0
      self.pub.publish(self.var)
    
    
################################ HOVEDFUNKSJON
################################

    
  def execute_callback(self, goal):
    
    self.rospy.loginfo("goal recieved:" + str(goal))
    self.cancelled = 0
    
    i = 0
    self.tik = time.time()
    self.stopp()
    
    if goal.figur == "sirkel_left":
      self.success = True
      
      while i < goal.reps:
      
        self._feedback.current_rep = i+1
        self._as.publish_feedback(self._feedback)
        
        self.var.linear.x = .0
        self.var.angular.z = 0.8
        self.pub.publish(self.var)
        for i in range(6):
          time.sleep(0.517)          
        
        i += 1
      
      
    if goal.figur == "sirkel_right":
      self.success = True
      
      while i < goal.reps:
      
        self._feedback.current_rep = i+1
        self._as.publish_feedback(self._feedback)
        
        self.var.linear.x = 0.0
        self.var.angular.z = -0.8
        self.pub.publish(self.var)
        for i in range(6):
          time.sleep(0.517)          
        
        i += 1
      
      
    #hvis firkant goal
    
    elif goal.figur == "firkant":
      
      while i < goal.reps:
      
        self._feedback.current_rep = i + 1
        self._as.publish_feedback(self._feedback)
        
        self.firkant()

        i += 1
      

    
    elif goal.figur == "straight":
      while True:
        self.var.linear.x = 0.5
        self.pub.publish(self.var)
        
        if self.cancelled == 1:
          break
    
    
    
    elif goal.figur == "line":
        
      self.var.linear.x = 0.4
      self.var.linear.x = 0.4
      self.pub.publish(self.var)
      time.sleep(0.3)
      self.var.linear.x = .30
      self.pub.publish(self.var)
      time.sleep(0.3)
      self.var.linear.x = .25
      self.pub.publish(self.var)
      
      s = 0
      while True:
                
          self.img = self.cam.read()[1]
          self.img = cv2.resize(self.img, self.dim, interpolation = cv2.INTER_NEAREST)
          hsv_img = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV).astype(np.float)
          mask = cv2.inRange(hsv_img, self.lower_yellow, self.upper_yellow)
          res = cv2.bitwise_and(self.img, self.img, mask = mask)

          m = cv2.moments(mask, False)

          try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']

          except ZeroDivisionError:
            cy, cx = self.height/2, self.width/2

          cv2.circle(res, (int(cx), int(cy)), 5, (0,0,255), -1)
          cv2.imshow('RES', res)

          error_x = cx - self.width / 2;

          self.var.angular.z = round(((((-error_x-240)/75)/1.7-0.25)*2)/1.5     , 2)
          self.var.linear.x = 0.25
          new = self.var.angular.z

          if self.var.angular.z < -0.89:
            self.var.angular.z = 0
            s += 1

          elif (new - self.prev > 0.05) or (new-self.prev < -0.05):
            self.prev = self.var.angular.z 
            self.pub.publish(self.var)
            s = 0

          if cv2.waitKey(1) == ord("q") or s > 50 or self.cancelled == 1:    
            break
                
                
    self.stopp()
    self.tok = time.time()
    
    if self.cancelled == 0:
      self._result.tid = self.tok-self.tik
    
    self.cam.release()
    cv2.destroyAllWindows()
      
    if self.success == True:
      rospy.loginfo('success, waiitng for new goal')
      self._as.set_succeeded(self._result)
      return True
    
    
      

if __name__ == '__main__':
  rospy.init_node('figur_server')
  rospy.loginfo('waiting for goal')
  Bil()
  rospy.spin()
