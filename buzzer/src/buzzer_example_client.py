#!/usr/bin/env python3
####################################################
# Imports
PKG = 'buzzer' # this package name
import roslib
import sys
import os
import rospy
from buzzer.srv import *
from std_msgs.msg import  String, Float64, Bool
####################################################
# Functions
def buzzer_client(requestedTime):
    """
    Function used to request a buzz for a certain amount of time
    @param float requestedTime (requested time)
    @return none
    """
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
        requestedTime = 2 # Buzz for 2 seconds
        buzzerState = buzzer_client(requestedTime)
