#! /usr/bin/env python3

from __future__ import print_function

import sys

import rospy
# Brings in the SimpleActionClient
import actionlib


import time
# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import motor.msg

def motor_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('motor_action_server', motor.msg.MotorAction)
    print('step 1 ok')
    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()
    print('server ok')
    # Creates a goal to send to the action server.
    goal = motor.msg.MotorGoal(order='lft')

    # Sends the goal to the action server.
    client.send_goal(goal)
    print('goal sent')


    # TEST => a remplacer par camera
    time.sleep(1)
    client.cancel_goal()
    time.sleep(2)
    
    goal = motor.msg.MotorGoal(order='rgt')
    client.send_goal(goal)
    print('goal sent')
    time.sleep(1)
    client.cancel_goal()
    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    try:
        print('on')
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('motor_client')
        print('init done')
        result = motor_client()
        #print("Result:", result.success)
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)

