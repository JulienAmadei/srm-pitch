#! /usr/bin/env python

import rospy
import actionlib
import servo.msg
import RPi.GPIO as GPIO
import time
from AB2.PCA9685 import PCA9685

class ServoAction(object):
    # create messages that are used to publish feedback/result
    _feedback = servo.msg.ServoFeedback()
    _result = servo.msg.ServoResult()   

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, servo.msg.ServoAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
      
    def execute_cb(self, goal):
        pwm = PCA9685(0x40)
        pwm.setPWMFreq(50)

        # In many RC servos a 1.5 ms pulse equates to a rotation of 90°
        #Set servo parameters
        HPulse = 1500  #Sets the initial Pulse
        HStep = 0      #Sets the initial step length
        VPulse = 1500  #Sets the initial Pulse
        VStep = 0      #Sets the initial step length
        rospy.loginfo('Resetting Horizontal & Vertical Servo')
        pwm.setServoPulse(0,HPulse)
        pwm.setServoPulse(1,VPulse)
        
        rospy.loginfo('Executing motion for H')
        print(goal.H)
        HPulse = goal.H*(2000/180)+1500
        if(HStep != 0):
            HPulse += HStep
            if(HPulse >= 2500): 
                HPulse = 2500
                if(HPulse <= 500):
                    HPulse = 500
        pwm.setServoPulse(0,HPulse)   
        time.sleep(0.02)     
    
        rospy.loginfo('Executing motion for V')
        print(goal.V)
        VPulse = goal.V*(2000/180)+1500
        if(VStep != 0):
            VPulse += VStep
            if(VPulse >= 2500): 
                VPulse = 2500
                if(VPulse <= 500):
                    VPulse = 500
        pwm.setServoPulse(1,VPulse)   
        time.sleep(0.02)
        
        # check that preempt has not been requested by the client        
        while not self._as.is_preempt_requested():
            time.sleep(0.01)
        rospy.loginfo('%s: Preempted' % self._action_name)
        self._as.set_preempted()
        success = True
        
if __name__ == '__main__':
    print("[SERVO PACKAGE] Action Server running.")
    rospy.init_node('servo_action_server')
    server = ServoAction(rospy.get_name())
    rospy.spin()
