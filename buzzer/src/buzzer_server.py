#!/usr/bin/env python3

####################################################
# Imports
from buzzer.srv import *
PKG = 'buzzer'
NAME = 'buzzer_server'
import roslib
import time
from std_msgs.msg import  String, Float64, Bool
import rospy 
import RPi.GPIO as GPIO
####################################################
# GPIO
BUZ = 4
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(BUZ,GPIO.OUT)
####################################################
# Functions
def buzz_for_time(req):
    """
    Function used to buzz for a certain amount of time
    @param float req (requested time)
    @return bool True (Is the buzzer done buzzing)
    """
    print(f"Buzzing for {req.buzzTime}")
    GPIO.output(BUZ,GPIO.HIGH)
    time.sleep(req.buzzTime)
    GPIO.output(BUZ,GPIO.LOW)
    print("Done")
    return BuzzerResponse(True)

def buzzer_server():
    """
    Function used to create the buzzer server
    """
    rospy.init_node(NAME)
    s = rospy.Service('buzzer', Buzzer, buzz_for_time)
    rospy.spin()


if __name__ == "__main__":
    print("[BUZZER PACKAGE] Buzzer Server running.")
    buzzer_server()
