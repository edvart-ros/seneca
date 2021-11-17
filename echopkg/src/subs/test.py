#! /usr/bin/env python

import rospy
from std_msgs.msg import Float32
from mDev import *
from geometry_msgs.msg import Twist

mdev = mDEV()

mdev.setServo('1', 150)
