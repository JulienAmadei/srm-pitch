# NeoPixel library strandtest example
# Author: Louis
#
# inputs ( time/int/, mode/int/, color/arrayRGB/)
# outputs ( end_of_task_bool ) 

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
        os.system("sudo python3 ~/test_ws/src/led/srv/blink_led_exe.py "+arg)
    elif req.mode == 1:
        os.system("sudo python3 ~/test_ws/src/led/srv/quarter_led_exe.py "+arg)
    time.sleep(tf)

    return True


def blink_led_server():
    rospy.init_node('blink_led_server')
    s = rospy.Service('led_blink', BlinkLED, blink)
    rospy.spin()

    

if __name__ == "__main__":
    blink_led_server()


