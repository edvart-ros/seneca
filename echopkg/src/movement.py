#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from echopkg.msg import FigurAction, FigurGoal
from echopkg.msg import color_msg
import actionlib
from std_msgs.msg import Float32



rospy.init_node('movement_node')
var = Twist()

client = actionlib.SimpleActionClient('/figur_action', FigurAction)
client.wait_for_server()

goal = FigurGoal()
goal.figur = str(input("hvilken figur vil du kjore?"))
goal.reps = int(input("Hvor mange repetisjoner?"))
client.send_goal(goal)

client.wait_for_result()
print('finished')