#!/usr/bin/env python3

# Import the necessary libraries
import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2
import numpy as np
from RobotHandGestures import utlis

##############################################################################

cropVals = 100,100,300,400 # StartPointY StartPointX h w
frameWidth = 640
frameHeight = 480
brightnessImage = 230

##############################################################################

utlis.initializeTrackBar()

def hand_main(data):
    br = CvBridge()
    img = br.imgmsg_to_cv2(data)
    imgResult = img.copy()

    imgBlur = cv2.GaussianBlur(img, (7, 7), 1)
    imgHSV = cv2.cvtColor(imgBlur, cv2.COLOR_BGR2HSV)
    trackBarPos = utlis.getTrackbarValues()
    imgMask, imgColorFilter = utlis.colorFilter(imgHSV,trackBarPos)

    imgCropped = imgMask[cropVals[1]:cropVals[2]+cropVals[1],cropVals[0]:cropVals[0]+cropVals[3]]
    imgResult = imgResult[cropVals[1]:cropVals[2] + cropVals[1], cropVals[0]:cropVals[0] + cropVals[3]]
    imgOpen =cv2.morphologyEx(imgCropped, cv2.MORPH_OPEN,np.ones((5,5),np.uint8))
    imgClosed = cv2.morphologyEx(imgOpen, cv2.MORPH_CLOSE, np.ones((10, 10), np.uint8))
    imgFilter = cv2.bilateralFilter(imgClosed, 5, 75, 75)
    imgContour,imgResult = utlis.getContours(imgFilter,imgResult)

    ## TO DISPLAY
    cv2.rectangle(img, (cropVals[0], cropVals[1]), (cropVals[0]+cropVals[3], cropVals[2]+cropVals[1]), (0, 255, 0), 2)
    stackedImage = utlis.stackImages(0.7,([img,imgMask,imgColorFilter],[imgCropped,imgContour,imgResult]))



    #imgBlank = np.zeros((512, 512, 3), np.uint8)
    #stackedImage = utlis.stackImages(0.7, ([img, imgBlank, imgBlank], [imgBlank, imgBlank, imgBlank]))

    cv2.imshow('Stacked Images', stackedImage)


    if cv2.waitKey(1) & 0xFF == ord('q'):
        break



def ros_main():
 
  # Tells rospy the name of the node.
  # Anonymous = True makes sure the node has a unique name. Random
  # numbers are added to the end of the name. 
  rospy.init_node('video_sub_py', anonymous=True)
   
  # Node is subscribing to the video_frames topic
  rospy.Subscriber('video_frames', Image, hand_main)
 
  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()
 
  # Close down the video stream when done
  cv2.destroyAllWindows()
  
if __name__ == '__main__':
  ros_main()
