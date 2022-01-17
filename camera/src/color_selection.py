#!/usr/bin/env python3
# Description:
# - Subscribes to real-time streaming video from your built-in webcam.
#
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
 
# Import the necessary libraries
import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np

cv2.namedWindow("HSV")
cv2.resizeWindow("HSV",640,240)
cv2.createTrackbar("HUE Min","HSV",0,179,empty)
cv2.createTrackbar("HUE Max","HSV",179,179,empty)
cv2.createTrackbar("SAT Min","HSV",0,255,empty)
cv2.createTrackbar("SAT Max","HSV",255,255,empty)
cv2.createTrackbar("VALUE Min","HSV",0,255,empty)
cv2.createTrackbar("VALUE Max","HSV",255,255,empty)


def empty(a):
    pass
    
def color_main(data):
 
  # Used to convert between ROS and OpenCV images
  br = CvBridge()
  

  # Output debugging information to the terminal
  rospy.loginfo("receiving video frame")
  
  # Convert ROS Image message to OpenCV image
  img = br.imgmsg_to_cv2(data)
  imgHsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
  h_min = cv2.getTrackbarPos("HUE Min","HSV")
  h_max = cv2.getTrackbarPos("HUE Max", "HSV")
  s_min = cv2.getTrackbarPos("SAT Min", "HSV")
  s_max = cv2.getTrackbarPos("SAT Max", "HSV")
  v_min = cv2.getTrackbarPos("VALUE Min", "HSV")
  v_max = cv2.getTrackbarPos("VALUE Max", "HSV")
  print(h_min)

  lower = np.array([h_min,s_min,v_min])
  upper = np.array([h_max,s_max,v_max])
  mask = cv2.inRange(imgHsv,lower,upper)
  result = cv2.bitwise_and(img,img, mask = mask)

  mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
  hStack = np.hstack([img,mask,result])
  #cv2.imshow('Original', img)
  #cv2.imshow('HSV Color Space', imgHsv)
  #cv2.imshow('Mask', mask)
  cv2.imshow('Result', result)
  cv2.imshow('Horizontal Stacking', hStack)
  
  cv2.waitKey(1)
      
def ros_main():
 
  # Tells rospy the name of the node.
  # Anonymous = True makes sure the node has a unique name. Random
  # numbers are added to the end of the name. 
  rospy.init_node('video_sub_py', anonymous=True)
   
  # Node is subscribing to the video_frames topic
  rospy.Subscriber('video_frames', Image, callback)
 
  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()
 
  # Close down the video stream when done
  cv2.destroyAllWindows()
  
if __name__ == '__main__':
  ros_main()
