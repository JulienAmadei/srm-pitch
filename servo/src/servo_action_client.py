#! /usr/bin/env python3

from __future__ import print_function
import sys
import rospy
import actionlib
import time
import servo.msg

def servo_client():
    client = actionlib.SimpleActionClient('servo_action_server', servo.msg.ServoAction)
    print('Client created !')
    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()
    print('Server launched !')
    # Creates a goal to send to the action server.
    goal = servo.msg.ServoGoal(H=20,V=10)
    # Sends the goal to the action server.
    client.send_goal(goal)
    print(f'Goal sent (H:{goal.H}, V:{goal.V})')

########### TODO ##########
# Retour valeur axe Z et valeur axe X ==> feedback 

    # TEST => a remplacer par camera
    time.sleep(5)
    client.cancel_goal()
    time.sleep(2)
    
    goal = servo.msg.ServoGoal(H=-10,V=-5)
    client.send_goal(goal)
    print(f'Goal sent (H:{goal.H}, V:{goal.V})')
    time.sleep(5)
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
        rospy.init_node('servo_client')
        print('Init done !')
        result = servo_client()
        #print("Result:", result.success)
    except rospy.ROSInterruptException:
        print("Program has been interrupted before completion", file=sys.stderr)

