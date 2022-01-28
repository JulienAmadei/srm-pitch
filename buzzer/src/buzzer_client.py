#!/usr/bin/env python3

PKG = 'buzzer' # this package name

import roslib

import sys
import os
import rospy

from buzzer.srv import *
from std_msgs.msg import  String, Int64, Bool

## Call for buzzer method depending of the type of object cast

def buzzer_client(requestedTime):
    # Block until the service is available
    rospy.wait_for_service('buzzer')
    try:
        buzzerProxy = rospy.ServiceProxy('buzzer', Buzzer)     
        resp = buzzerProxy.call(BuzzerRequest(requestedTime))

        if not resp.buzzState == True:
            raise Exception("End of Call failure, returned resp was %f"%resp.buzzerState)
        return resp.buzzState

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
        rospy.init_node('buzzer_client')
        requestedTime = 2
        buzzerState = buzzer_client(requestedTime)
