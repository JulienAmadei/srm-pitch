#!/usr/bin/env python3

# Import the necessary libraries
import rospy # Python library for ROS
import cv2 # OpenCV library
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import numpy as np


cv2.namedWindow('Result')

def color_main(data):
 
    # Used to convert between ROS and OpenCV images
    br = CvBridge()
    
    # Output debugging information to the terminal
    rospy.loginfo("receiving video frame")
    
    # Convert ROS Image message to OpenCV image
    img = br.imgmsg_to_cv2(data)
    imgHsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    h_min = 0
    h_max = 20
    s_min = 50
    s_max = 160
    v_min = 100
    v_max = 255
    
    lower = np.array([h_min,s_min,v_min])
    upper = np.array([h_max,s_max,v_max])
    mask = cv2.inRange(imgHsv,lower,upper)
    result = cv2.bitwise_and(img,img, mask = mask)
    edges = cv2.findContours(result,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
  
    imgCon = cv2.cvtColor(imgHsv,cv2.COLOR_HSV2BGR)
    bigCon = 0
    myCounter=0
    myPos = np.zeros(4)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if (area>1000):
            cv2.drawContours(imgCon, cnt, -1, (255, 0, 255), 3)
            cv2.drawContours(imgMatch, cnt, -1, (255, 0, 255), 3)
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
            # APPROXIMATED BOUNDING BOX
            x, y, w, h = cv2.boundingRect(approx)
            ex=10
            cv2.rectangle(imgCon, (x-ex, y-ex), (x + w+ex, y + h+ex), (0,255,0), 5);
            # CONVEX HULL & CONVEXITY DEFECTS OF THE HULL
            hull = cv2.convexHull(cnt, returnPoints=False)
            defects = cv2.convexityDefects(cnt, hull)
            bigCon += 1

            for i in range(defects.shape[0]):  # calculate the angle
                s, e, f, d = defects[i][0]
                start = tuple(cnt[s][0])
                end = tuple(cnt[e][0])
                far = tuple(cnt[f][0])
                a = math.sqrt((end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2)
                b = math.sqrt((far[0] - start[0]) ** 2 + (far[1] - start[1]) ** 2)
                c = math.sqrt((end[0] - far[0]) ** 2 + (end[1] - far[1]) ** 2)
                angle = math.acos((b ** 2 + c ** 2 - a ** 2) / (2 * b * c))  # cosine theorem
                if angle <= math.pi // 1.7:  # angle less than  degree, treat as fingers
                    myPos[myCounter] = far[0]
                    myCounter += 1
                    cv2.circle(imgCon, far, 5, [0, 255, 0], -1)
                    cv2.circle(imgMatch, far, 5, [0, 255, 0], -1)

            ## SENDING COMMANDS BASED ON FINGERS
            if (myCounter==4): sendData([1,1,1,1,1]);FingerCount="Five"
            elif (myCounter == 3): sendData([1, 1, 1, 1, 0]);FingerCount="Four"
            elif (myCounter == 2):sendData([0, 1, 1, 1, 0]);FingerCount="Three"
            elif (myCounter == 1):sendData([0, 0, 1, 1, 0]);FingerCount="Two"
            elif (myCounter == 0):
                aspectRatio = w/h
                if aspectRatio < 0.6:
                    sendData([0, 0, 0, 1, 0]);FingerCount="One"
                else: sendData([0, 0, 0, 0, 0]);FingerCount="Zero"
            cv2.putText(imgMatch,FingerCount,(50,50),cv2.FONT_HERSHEY_COMPLEX,1,(0,0,255),2)
    cv2.imshow(imgCon)
    cv2.imshow(imgMatch)
    
    cv2.waitKey(1)
  
def hand_main():

    br = CvBridge()
    rospy.loginfo("receiving video frame")
    img = br.imgmsg_to_cv2(data)
    imgResult = img.copy()

    imgBlur = cv2.GaussianBlur(img, (7, 7), 1)
    imgHSV = cv2.cvtColor(imgBlur, cv2.COLOR_BGR2HSV)
    trackBarPos = getTrackbarValues()
    imgMask, imgColorFilter = colorFilter(imgHSV,trackBarPos)

    imgCropped = imgMask[cropVals[1]:cropVals[2]+cropVals[1],cropVals[0]:cropVals[0]+cropVals[3]]
    imgResult = imgResult[cropVals[1]:cropVals[2] + cropVals[1], cropVals[0]:cropVals[0] + cropVals[3]]
    imgOpen =cv2.morphologyEx(imgCropped, cv2.MORPH_OPEN,np.ones((5,5),np.uint8))
    imgClosed = cv2.morphologyEx(imgOpen, cv2.MORPH_CLOSE, np.ones((10, 10), np.uint8))
    imgFilter = cv2.bilateralFilter(imgClosed, 5, 75, 75)
    imgContour,imgResult = getContours(imgFilter,imgResult)

    ## TO DISPLAY
    cv2.rectangle(img, (cropVals[0], cropVals[1]), (cropVals[0]+cropVals[3], cropVals[2]+cropVals[1]), (0, 255, 0), 2)
    stackedImage = stackImages(0.7,([img,imgMask,imgColorFilter],[imgCropped,imgContour,imgResult]))



    #imgBlank = np.zeros((512, 512, 3), np.uint8)
    #stackedImage = stackImages(0.7, ([img, imgBlank, imgBlank], [imgBlank, imgBlank, imgBlank]))

    cv2.imshow('Stacked Images', stackedImage)

    cv2.waitKey(1)
      
def ros_main():
 
  # Tells rospy the name of the node.
  # Anonymous = True makes sure the node has a unique name. Random
  # numbers are added to the end of the name. 
  rospy.init_node('rock_paper_scisors_py', anonymous=True)
   
  # Node is subscribing to the video_frames topic
  rospy.Subscriber('video_frames', Image, color_main)
 
  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()
 
  # Close down the video stream when done
  cv2.destroyAllWindows()
  
if __name__ == '__main__':
  ros_main()
