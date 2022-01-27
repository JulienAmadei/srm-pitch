#! /usr/bin/env python3

from __future__ import print_function
import sys
import rospy
import actionlib
import time
import servo.msg
from sensor_msgs.msg import Image # Image is the message type
import cv2 # OpenCV library
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Image
import time

def servo_client():
    client = actionlib.SimpleActionClient('servo_action_server', servo.msg.ServoAction)
    print('Servo client created !')
    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()
    print('Servo Server found !')
    # Creates a goal to send to the action server.
    goal = servo.msg.ServoGoal(H=20,V=10)
    # Sends the goal to the action server.
    client.send_goal(goal)
    print(f'Goal sent (H:{goal.H}, V:{goal.V})')
    # TEST => a remplacer par camera
    time.sleep(5)
    client.cancel_goal()
    time.sleep(2)
    
    goal = servo.msg.ServoGoal(H=-10,V=-50)
    client.send_goal(goal)
    print(f'Goal sent (H:{goal.H}, V:{goal.V})')
    time.sleep(5)
    client.cancel_goal()
    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

def callback(data):
 
  # Used to convert between ROS and OpenCV images
  br = CvBridge()
 
  # Output debugging information to the terminal
  rospy.loginfo("receiving video frame")
   
  # Convert ROS Image message to OpenCV image
  current_frame = br.imgmsg_to_cv2(data)
   
  # Display image
  cv2.imshow("camera", current_frame)
   
  cv2.waitKey(1)
        
if __name__ == '__main__':
    try:
        print('on')
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('rough_client')
        print('Init done !')
        img = rospy.Subscriber('video_frames', Image, callback)
        time.sleep(2)
        result = servo_client()	
    	
    except rospy.ROSInterruptException:
        print("Program has been interrupted before completion", file=sys.stderr)
    # Close down the video stream when done
    cv2.destroyAllWindows()
