#!/usr/bin/env python
#########################
# Imports
from turtle import end_fill
import rospy
import RPi.GPIO as GPIO
from AB2.TRSensors import TRSensor
from std_msgs.msg import Bool
import time

#########################
# Functions
def talker():
    """
    Generic Talker
    """
    pub_Line = rospy.Publisher('ir_line_sensor', Bool, queue_size = 10) 
    rospy.init_node('ir_line_sensor_talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        position,Sensors = TR.readLine()
        if(Sensors[0] >900 and Sensors[1] >900 and Sensors[2] >900 and Sensors[3] >900 and Sensors[4] >900):
            line_status = False
        else:
            line_status = True
        pub_Line.publish(line_status)
        rate.sleep()

# To execute when invoked directly 
if __name__ == '__main__':
    print("[IR_LINE_SENSOR PACKAGE] IR Line Sensor Talker running.")
    TR = TRSensor()
    # PINS and INIT
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    TR.calibrate()
    try:
        # Run code
        talker()
    except rospy.ROSInterruptException:
        pass
