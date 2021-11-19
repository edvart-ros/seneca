#! /usr/bin/env python

import rospy
from mDev import *
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
mdev = mDEV()
from std_msgs.msg import Float32

import time

avstand = Float32()
mdev = mDEV()



def callback(request):
    response = TriggerResponse()
    mdev.setServo('2', 90)
    avstand = rospy.wait_for_message("/cmd_vel", Float32)
    time.sleep(0.7)

    if avstand < 1:
        response.success = True
    
    else:
        response.success = False

    return TriggerResponse()




rospy.init_node("subpubtingen")
my_service = rospy.Service("/crash_front_service", Trigger, callback)
rospy.spin()

