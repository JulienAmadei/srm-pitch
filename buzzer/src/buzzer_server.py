#!/usr/bin/env python

from buzzer.srv import *
PKG = 'buzzer'
NAME = 'buzzer_server'

import roslib
import time

from std_msgs.msg import  String, Int64, Bool
import rospy 

import RPi.GPIO as GPIO

BUZ = 4

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(BUZ,GPIO.OUT)

def buzz_for_time(req):
    print(f"Buzzing for {req.buzzTime}")
    GPIO.output(BUZ,GPIO.HIGH)
    time.sleep(req.buzzTime)
    GPIO.output(BUZ,GPIO.LOW)
    print("Done")
    return BuzzerResponse(True)

def buzzer_server():
    rospy.init_node(NAME)
    s = rospy.Service('buzzer', Buzzer, buzz_for_time)
    rospy.spin()

if __name__ == "__main__":
    print("[Buzzer server] Please keep this running in a separate tab.")
    buzzer_server()
