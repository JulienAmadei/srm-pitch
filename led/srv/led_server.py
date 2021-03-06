#!/usr/bin/env python

from __future__ import print_function
from led.srv import BlinkLED,BlinkLEDResponse
import time
import rospy
import os



def blink(req):

    tf = req.time
    RGB = req.color
    R = RGB[0]
    G = RGB[1]
    B = RGB[2]
    arg = str(tf)+" "+str(R)+" "+str(G)+" "+str(B)
    if req.mode == 0:
        os.system("sudo python3 ~/pitch_ws/src/srm-pitch/led/srv/blink_led_exe.py "+arg)
        time.sleep(1)
    elif req.mode == 1:
        os.system("sudo python3 ~/pitch_ws/src/srm-pitch/led/srv/quarter_led_exe.py "+arg)
        time.sleep(1)
    elif req.mode == 2:
        os.system("sudo python3 ~/pitch_ws/src/srm-pitch/led/srv/static_led_exe.py "+arg)
        time.sleep(1)
    return True


def blink_led_server():
    rospy.init_node('led_server')
    s = rospy.Service('led_blink', BlinkLED, blink)
    rospy.spin()

    

if __name__ == "__main__":
    print("[LED PACKAGE] Server Running.")
    blink_led_server()


