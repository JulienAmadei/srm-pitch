#!/usr/bin/env python3

import rospy
import RPi.GPIO as GPIO
import time
from std_msgs.msg import Bool
def talker():
    pub_L = rospy.Publisher('ir_topL', Bool, queue_size = 10) 
    pub_R = rospy.Publisher('ir_topR', Bool, queue_size = 10) 
    rospy.init_node('ir_top_talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        DL_status = GPIO.input(DL)
        DR_status = GPIO.input(DR)

        # Publishes a string to our messages topic
        pub_L.publish(DL_status)
        pub_R.publish(DR_status)
        # Sleeps just long enough to maintain the desired rate through the loop. 
        rate.sleep()

# To execute when invoked directly 
if __name__ == '__main__':
    print("[IR Proximity Sensor listener] Running.")
    # PINS and INIT
    DR = 16
    DL = 19
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(DR,GPIO.IN,GPIO.PUD_UP)
    GPIO.setup(DL,GPIO.IN,GPIO.PUD_UP)
    try:
        # Run code
        talker()
    except rospy.ROSInterruptException:
        pass
