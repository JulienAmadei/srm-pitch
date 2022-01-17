#!/usr/bin/env python3

import cv2
import numpy as np
import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
from matplotlib import pyplot as plt

def edge_main(data):

    br = CvBridge()
    rospy.loginfo("receiving video frame")
    img = br.imgmsg_to_cv2(data)
    edges = cv2.Canny(img,100,200)
    cv2.imshow("original",img)

    cv2.imshow("edges",edges)
    cv2.waitKey(1)
        
def ros_main():

  rospy.init_node('video_sub_py', anonymous=True)
  
  # Node is subscribing to the video_frames topic
  rospy.Subscriber('video_frames', Image, edge_main)
 
  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()
 
  # Close down the video stream when done
  cv2.destroyAllWindows()
  
if __name__ == '__main__':
  ros_main()
