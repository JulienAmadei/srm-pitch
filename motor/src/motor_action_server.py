#! /usr/bin/env python

import rospy

import actionlib

import motor.msg

import RPi.GPIO as GPIO
import time
from AB2.AlphaBot2 import AlphaBot2



class MotorAction(object):
    # create messages that are used to publish feedback/result
    _feedback = motor.msg.MotorFeedback()
    _result = motor.msg.MotorResult()

    

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, motor.msg.MotorAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
      
    def execute_cb(self, goal):
    
        Ab = AlphaBot2()
    	
        # helper variables
        #success = True
        
        # append the seeds for the fibonacci sequence
        #self._feedback.sequence = []


        # publish info to the console for the user
        rospy.loginfo('Executing motion')
        # start executing the action
        
        # check that preempt has not been requested by the client
        print(goal.order)
        if goal.order == 'fwd':
            Ab.forward();
        elif goal.order == 'bwd':
            Ab.backward();
        elif goal.order == 'rgt':
            Ab.right();
        elif goal.order == 'lft':
            Ab.left();
        else:
            rospy.loginfo('unknown order')
            
            #self._feedback.sequence.append(self._feedback.sequence[i] + self._feedback.sequence[i-1])
            # publish the feedback
            #self._as.publish_feedback(self._feedback)
            # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
            #r.sleep()
          
        while not self._as.is_preempt_requested():
            time.sleep(0.01)
        rospy.loginfo('%s: Preempted' % self._action_name)
        self._as.set_preempted()
        Ab.stop();
        success = True
        
if __name__ == '__main__':
    print("[Motor action server] Running.")
    rospy.init_node('motor_action_server')
    server = MotorAction(rospy.get_name())
    rospy.spin()
