#! /usr/bin/env python

import rospy
from echopkg.msg import FigurAction, FigurFeedback, FigurResult
import actionlib
import time
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID

#jeg vil lage en server som tar en string (figur), og en int (repetisjoner) inn som "goal".
#action server skal gi navaerende repetisjon som feedback
#action server skal returnere tid det tok a oppna malet som resultat.

class Bil(object):

  _feedback = FigurFeedback()
  
  def __init__(self):
    self._as = actionlib.SimpleActionServer('/figur_action', FigurAction, self.execute_callback, auto_start = False) #starter action server
    #self.sub = rospy.Subscriber('/echo', Float32, self.sub_cb, queue_size = 1)                          #subscriber til echo
    self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)                                      #lager publisher til cmd_vel
    self._feedback = FigurFeedback()                                                                    #dewfinerer feedback-object
    self._result = FigurResult()                                                                       #definerer resultat-object
    self._as.start()                                                                                    #starter server
    #self.ctrl_c = False
    self.rospy = rospy
    self.avstand = Float32()
    self.var = Twist()
    self.success = False
    self.cancelled = 3
    
    
    
#############################  HJELPEFUNKSJONER, TOPIC CALLBACKS
#############################


  
  def sub_cb(self, msg):
    dist = msg.data
    if dist < 15 and self.cancelled == 0:
      self.rospy.loginfo("goal preempted")
      self._as.set_aborted(self._result)
      self.success = False
      self.stopp()
      self.cancelled = 1
      return self.cancelled
      
    
    
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
    
    self.rospy.loginfo("action server called with argument:" + str(goal))
    
    self.sub = rospy.Subscriber('/echo', Float32, self.sub_cb, queue_size = 1)
    self.cancelled = 0

    
    
    i = 0
    tik = time.time()
    self.stopp()

    
    if goal.figur == "sirkel":
      self.success = True
      
      while i < goal.reps:
      
      
        self._feedback.current_rep = i+1
        self._as.publish_feedback(self._feedback)
        
        self.var.linear.x = .8
        self.var.angular.z = 0.8
        self.pub.publish(self.var)
        for i in range(6):
          time.sleep(0.517)          
          
        
        i += 1
      
      tok = time.time()
      
      
      
    #om firkant goal
    
    elif goal.figur == "firkant":
      
      tik = time.time()
      
      while i < goal.reps:
      
        self._feedback.current_rep = i + 1
        self._as.publish_feedback(self._feedback)
        
        self.firkant()

        i += 1
      
      
      tok = time.time()  #maler tid igjen
    
    self.stopp()
    self._result.tid = tok-tik
    
      
    if self.success == True:
      rospy.loginfo('success')
      self._as.set_succeeded(self._result)
      return True
      

if __name__ == '__main__':
  rospy.loginfo('server started')
  rospy.init_node('figur_server')
  Bil()
  rospy.spin()

      
      
      
    
    
