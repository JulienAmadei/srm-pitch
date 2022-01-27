#!/usr/bin/env python3
 
# Import the necessary libraries
import rospy # Python library for ROS
import cv2 # OpenCV library
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import numpy as np
from ColorRecognitionService import *

def color_main(data):
 
  # Used to convert between ROS and OpenCV images
  br = CvBridge()
  
  # Output debugging information to the terminal
  rospy.loginfo("receiving video frame")
  
  # Convert ROS Image message to OpenCV image
  img = br.imgmsg_to_cv2(data)
  imgHsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
  h_min = 80
  h_max = 100
  s_min = 0
  s_max = 255
  v_min = 0
  v_max = 255

  lower = np.array([h_min,s_min,v_min])
  upper = np.array([h_max,s_max,v_max])
  mask = cv2.inRange(imgHsv,lower,upper)
  result = cv2.bitwise_and(img,img, mask = mask)

  cv2.imshow('Result', result)
  cv2.waitKey(1)
      
def ros_main():
 
  # Tells rospy the name of the node.
  # Anonymous = True makes sure the node has a unique name. Random
  # numbers are added to the end of the name. 
  rospy.init_node('color_selection_py', anonymous=True)
   
  # Node is subscribing to the video_frames topic
  rospy.Subscriber('video_frames', Image, color_main)
 
  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()
 
  # Close down the video stream when done
  cv2.destroyAllWindows()
  
if __name__ == '__main__':
  ros_main()
