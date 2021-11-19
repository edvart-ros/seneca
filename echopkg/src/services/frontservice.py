#! /usr/bin/env python

import rospy
from mDev import *
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from std_msgs.msg import Float32

import time

avstand = Float32()


def callback(request):
    response = TriggerResponse()
    mdev.setServo('2', 90)
    time.sleep(0.7)
    avstand = rospy.wait_for_message("/cmd_vel", Float32)


    if avstand < 1:
        response.success = True
        response.message = "crash"
    
    else:
        response.success = False
        response.message = "klart"

    return TriggerResponse()




rospy.init_node("crash_front_server")
my_service = rospy.Service("/crash_front_service", Trigger, callback)
rospy.spin()

