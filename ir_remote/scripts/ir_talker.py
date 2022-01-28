## Talker that publishes a message to the remote_input topic.

#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import String 
import RPi.GPIO as GPIO

IR = 17
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(IR,GPIO.IN)

def getkey():
	if GPIO.input(IR) == 0:
		count = 0
		while GPIO.input(IR) == 0 and count < 200:  #9ms
			count += 1
			time.sleep(0.00006)
		if(count < 10):
			return;
		count = 0
		while GPIO.input(IR) == 1 and count < 80:  #4.5ms
			count += 1
			time.sleep(0.00006)

		idx = 0
		cnt = 0
		data = [0,0,0,0]
		for i in range(0,32):
			count = 0
			while GPIO.input(IR) == 0 and count < 15:    #0.56ms
				count += 1
				time.sleep(0.00006)
				
			count = 0
			while GPIO.input(IR) == 1 and count < 40:   #0: 0.56mx
				count += 1                               #1: 1.69ms
				time.sleep(0.00006)
				
			if count > 7:
				data[idx] |= 1<<cnt
			if cnt == 7:
				cnt = 0
				idx += 1
			else:
				cnt += 1
		if data[0]+data[1] == 0xFF and data[2]+data[3] == 0xFF:  #check
			return data[2]


def talker():
    pub = rospy.Publisher('remote_input', String, queue_size = 1) 
    rospy.init_node('remote_talker', anonymous=True)
    rate = rospy.Rate(500) # 5000hz
    while not rospy.is_shutdown():
        try:
            key = getkey()            
            # Row 1
            if key == None:
                msg = "None (None)"
            if key == 0x45:
                msg = "CH- (0x45)"
            if key == 0x46:
                msg = "CH (0x46)"
            if key == 0x47:
                msg = "CH+ (0x47)"
            
            # Row 2
            if key == 0x44:
                msg = "|<< (0x44)"
            if key == 0x40:
                msg = ">>| (0x40)"
            if key == 0x43:
                msg = ">|| (0x43)"   
            
            # Row 3
            if key == 0x07:
                msg = "- (0x07)"
            if key == 0x15:
                msg = "+ (0x15)"
            if key == 0x09:
                msg = "EQ (0x09)"
            
            # Row 4
            if key == 0x16:
                msg = "0 (0x16)"
            if key == 0x19:
                msg = "100+ (0x19)"
            if key == 0x0D:
                msg = "200+ (0x0D)"

            # Row 5
            if key == 0x0C:
                msg = "1 (0x0C)"
            if key == 0x18:
                msg = "2 (0x18)"
            if key == 0x5E:
                msg = "3 (0x5E)"

            # Row 6
            if key == 0x08:
                msg = "4 (0x08)"
            if key == 0x1C:
                msg = "5 (0x1C)"
            if key == 0x5A:
                msg = "6 (0x5A)"
            
            # Row 7
            if key == 0x42:
                msg = "7 (0x42)"
            if key == 0x52:
                msg = "8 (0x52)"
            if key == 0x4A:
                msg = "9 (0x4A)"

            # None
            if key == None:
                msg = msg

            if key != None:
                # Prints the messages to screen, write them to the Node's log file, and write them to rosout.
                rospy.loginfo(msg)
                # Publishes a string to our messages topic
                pub.publish(msg)
                       
		#if key != None:
		
		    # time.sleep(1)
		
		# Sleeps just long enough to maintain the desired rate through the loop. 
            rate.sleep()
        except KeyboardInterrupt:
            GPIO.cleanup()
            
	

# To execute when invoked directly 
if __name__ == '__main__':
    print("[IR Remote listener] Running.")
    try:
        msg = ""
        talker()
    except rospy.ROSInterruptException:
        pass

