#!/usr/bin/env python3

# Import the necessary libraries
import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2
import numpy as np
import math

##############################################################################

cropVals = 100,100,300,400 # StartPointY StartPointX h w

##############################################################################

def mask_values():
    h_min = 0
    h_max = 26
    s_min = 50
    s_max = 160
    v_min = 30
    v_max = 255
    vals = h_min,s_min,v_min,h_max,s_max,v_max
    return vals


def colorFilter(img, vals):
    lower_blue = np.array([vals[0],vals[1], vals[2]])
    upper_blue = np.array([vals[3], vals[4], vals[5]])
    mask = cv2.inRange(img, lower_blue, upper_blue)
    imgColorFilter = cv2.bitwise_and(img, img, mask=mask)
    ret, imgMask = cv2.threshold(mask, 127, 255, 0)
    return imgMask,imgColorFilter


def sendData(fingers):

    string = "$"+str(int(fingers[0]))+str(int(fingers[1]))+str(int(fingers[2]))+str(int(fingers[3]))+str(int(fingers[4]))
    try:
       ser.write(string.encode())
       print(string)
    except:
        pass

def getContours(imgCon,imgMatch):

    contours, hierarchy = cv2.findContours(imgCon, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    imgCon = cv2.cvtColor(imgCon,cv2.COLOR_GRAY2BGR)
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
    return imgCon,imgMatch

def stackImages(scale,imgArray):
    rows = len(imgArray)
    cols = len(imgArray[0])
    rowsAvailable = isinstance(imgArray[0], list)
    width = imgArray[0][0].shape[1]
    height = imgArray[0][0].shape[0]
    if rowsAvailable:
        for x in range ( 0, rows):
            for y in range(0, cols):
                if imgArray[x][y].shape[:2] == imgArray[0][0].shape [:2]:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (0, 0), None, scale, scale)
                else:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (imgArray[0][0].shape[1], imgArray[0][0].shape[0]), None, scale, scale)
                if len(imgArray[x][y].shape) == 2: imgArray[x][y]= cv2.cvtColor( imgArray[x][y], cv2.COLOR_GRAY2BGR)
        imageBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imageBlank]*rows
        hor_con = [imageBlank]*rows
        for x in range(0, rows):
            hor[x] = np.hstack(imgArray[x])
        ver = np.vstack(hor)
    else:
        for x in range(0, rows):
            if imgArray[x].shape[:2] == imgArray[0].shape[:2]:
                imgArray[x] = cv2.resize(imgArray[x], (0, 0), None, scale, scale)
            else:
                imgArray[x] = cv2.resize(imgArray[x], (imgArray[0].shape[1], imgArray[0].shape[0]), None,scale, scale)
            if len(imgArray[x].shape) == 2: imgArray[x] = cv2.cvtColor(imgArray[x], cv2.COLOR_GRAY2BGR)
        hor= np.hstack(imgArray)
        ver = hor
    return ver

def hand_main(data):
    br = CvBridge()
    img = br.imgmsg_to_cv2(data)
    imgResult = img.copy()

    imgBlur = cv2.GaussianBlur(img, (7, 7), 1)
    imgHSV = cv2.cvtColor(imgBlur, cv2.COLOR_BGR2HSV)
    mask = mask_values()
    imgMask, imgColorFilter = colorFilter(imgHSV,mask)

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
  rospy.init_node('video_sub_py', anonymous=True)
   
  # Node is subscribing to the video_frames topic
  rospy.Subscriber('video_frames', Image, hand_main)
 
  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()
 
  # Close down the video stream when done
  cv2.destroyAllWindows()
  
if __name__ == '__main__':
  ros_main()
