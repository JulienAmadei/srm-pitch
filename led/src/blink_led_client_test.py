# NeoPixel library strandtest example
# Author: Louis
#
# inputs ( time/int/, mode/int/, color/arrayRGB/)
# outputs ( end_of_task_bool ) 

#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from led.srv import *


def led_blink_client(time, mode, color):
    rospy.wait_for_service('led_blink')
    try:
        blink = rospy.ServiceProxy('led_blink', BlinkLED)
        blink.call(BlinkLEDRequest(time, mode, color))
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)



if __name__ == "__main__":
    rospy.init_node('led_client')
    time = 10
    mode = 2
    color = [255,255,255]
    led_blink_client(time, mode, color)
