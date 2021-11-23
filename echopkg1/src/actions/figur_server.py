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
    
    # initialiserer action server, echosub og cmdpub
    self.done = 3   #variabel for aa vite hvilket stadie servern er i
    self._as = actionlib.SimpleActionServer('/figur_action', FigurAction, self.execute_callback, auto_start = False) #starter action server
    self.sub = rospy.Subscriber('/echo', Float32, self.sub_cb, queue_size = 1)                                    #subscriber til echo
    self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)                                                 #lager publisher til cmd_vel
    
    #starter action-server og lager objekter for feedback og resultat
    self._feedback = FigurFeedback()                                                                              #definerer feedback-object
    self._result = FigurResult()                                                                                  #definerer resultat-object
    self._as.start()                                                                                              #starter server
    
    # flere variabler
    self.ctrl_c = False
    self.rospy = rospy
    self.var = Twist()
    self.success = False

    #initialiserer opencv og tidsvariabler
    self.cv2 = cv2
    self.tik = time.time()
    self.tok = time.time()
    
    
    
    
#############################  HJELPEFUNKSJONER, TOPIC CALLBACKS
#############################



  
  def sub_cb(self, msg):      #callback-funksjon for echo
    dist = msg.data
    if dist < 5 and self.done == 0:
      self.rospy.loginfo("obstacle detected, distance: "+ str(dist))
      self.tok = time.time()
      self._result.tid = self.tok-self.tik
      self._as.set_aborted(self._result)    #kansellerer
      self.success = False
      self.stopp()
      self.done = 1
      return False
      
  def startup(self):            #funksjon for aa faa bilen i gang og trappe ned i hasthighet
      self.var.linear.x = 0.4
      self.pub.publish(self.var)
      time.sleep(0.5)
      self.var.linear.x = .30
      self.pub.publish(self.var)
      time.sleep(1)
      self.var.linear.x = .25
      self.pub.publish(self.var)
    
  def stopp(self):             #stopper bilen
    self.var.linear.x = 0
    self.var.angular.z = 0
    self.pub.publish(self.var)
    
    
    
  def firkant(self):          #kjorer bilen i en firkant
    self.stopp()
    time.sleep(0.1)
    self.success = True
    for i in range(4):
      if self.done == 1:
        break
      self.var.linear.x = 0.5
      self.pub.publish(self.var)
      time.sleep(.7)     
      if self.done == 1:
        break
      self.var.angular.z = 0.7
      self.pub.publish(self.var)
      time.sleep(1.1)
      if self.done == 1:
        break
      self.var.angular.z = 0
      self.pub.publish(self.var)
    
    
################################ HOVEDFUNKSJON
################################

    
    
    
  def execute_callback(self, goal):           #funksjonen som kalles paa av klient
    
    self.rospy.loginfo("goal recieved:" + str(goal))
    self.done = 0
    
    i = 0
    self.tik = time.time()      #tar opp tiden for aa senere regne ut tid brukt
    self.stopp()
    
    ### if-funksjoner for ulike goals
    ###
    
    if goal.figur == "sirkel_left":        
      self.success = True
      
      while i < goal.reps:
        self._feedback.current_rep = i+1
        self._as.publish_feedback(self._feedback)
        
        self.var.linear.x = 0.8
        self.var.angular.z = 0.8
        self.pub.publish(self.var)
        
        for i in range(12):
          time.sleep(0.32)
          if self.done == 1:
            break          
        
        if self.done == 1:
          break
        i += 1
      
      
    if goal.figur == "sirkel_right":      #if-setning for goal
      self.success = True
      
      while i < goal.reps:
      
        self._feedback.current_rep = i+1
        self._as.publish_feedback(self._feedback)   #publiserer feedback
        
        self.var.linear.x = 0.8
        self.var.angular.z = -0.8
        self.pub.publish(self.var)
        
        for i in range(12):
          time.sleep(0.32)
          if self.done == 1:
            break         
        
        if self.done == 1:    
          break
        i += 1
      
      
    
    elif goal.figur == "firkant":      #if-setning for goal
      
      while i < goal.reps:
      
        self._feedback.current_rep = i + 1
        self._as.publish_feedback(self._feedback)  #publiserer feedback
        
        self.firkant()

        i += 1
      

    
    elif goal.figur == "straight":    #if-setning for goal (uten feedback)
      while True:
        self.var.linear.x = 0.5
        self.pub.publish(self.var)
        
        if self.done == 1:
          break
    
    
    
    elif goal.figur == "line":    #if-setning for goal (uten feedback)
      self.success = True
      self.startup()
      
      cam = self.cv2.VideoCapture(0)   #velger webkamera
      for n in range(5):               #henter bilder fra kameraet (i en kort loop for aa unngaa en error)
        img = cam.read()[1]  
        
      width = int(img.shape[1] * 15 /100)        #lager variabler for dimensjonering
      height = int(img.shape[0] * 15 /100)
      dim = (width, height)
      height, width, channels = img.shape
      lower_yellow = np.array([20, 100, 100])    #velger hsv-omraade for fargen gul
      upper_yellow = np.array([50, 255, 255])
      prev = 0
      
      while True:

          img = cam.read()[1]                    #laster inn bilde fra kamera
          img = self.cv2.resize(img, dim, interpolation = self.cv2.INTER_NEAREST)  #dimensjonerer bilde for bedre ytelse
          hsv_img = self.cv2.cvtColor(img, self.cv2.COLOR_BGR2HSV).astype(np.float)  #konverterer bilde fra bgr til hsv
          mask = self.cv2.inRange(hsv_img, lower_yellow, upper_yellow)              #lager en maske med pixlene innenfor grensen vaar
          res = self.cv2.bitwise_and(img, img, mask = mask)                        #kombinerer masken og det originale bildet
          m = self.cv2.moments(mask, False)                                        #finner centroid av de gule tingene i bildet

          try:                                                                     #finner koordinatene til centroid
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']

          except ZeroDivisionError:
            cy, cx = height/2, width/2

          self.cv2.circle(res, (int(cx), int(cy)), 5, (0,0,255), -1)            #tegner sirkelen oppaa bildet vaart
          self.cv2.imshow('RES', res)


          
          error_x = cx - width / 2;        #regner ut hvor bilen skal kjoore i forhold til hvor centroiden er 

          self.var.angular.z = round(((((-error_x-240)/75)/1.7-0.25)*2)/1.5     , 2)
          self.var.linear.x = 0.25
          new = self.var.angular.z

          if self.var.angular.z < -0.89:
            self.var.angular.z = 0

          elif (new - prev > 0.05) or (new-prev < -0.05):       #legger inn en kontroll som reduserer smaa justeringer av styringen
            prev = self.var.angular.z 
            self.pub.publish(self.var)
          
          
          if self.cv2.waitKey(1) == ord("q") or self.done == 1:      #sorger for at programmet kan stoppes og kameraet kan "releases"
            cam.release()
            break
            
                
    self.stopp()                                                #stopper bilen
    self.tok = time.time()
    
    if self.done == 0:
      self._result.tid = self.tok-self.tik                      #regner ut tid brukt
    
    self.cv2.destroyAllWindows()
      
    if self.success == True:                              #logger suksess og publiserer result 
      rospy.loginfo('success, waiitng for new goal')
      self.done = 1
      self._as.set_succeeded(self._result)
      return True
    


if __name__ == '__main__':
  rospy.init_node('figur_server')
  rospy.loginfo('waiting for goal')
  Bil()
  rospy.spin()
