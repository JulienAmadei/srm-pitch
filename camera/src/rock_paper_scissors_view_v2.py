#!/usr/bin/env python3

# Import the necessary libraries
import rospy # Python library for ROS
import cv2
import numpy as np
import math
from std_msgs.msg import Int16 # Image is the message type

def automatic_brightness_and_contrast(image, clip_hist_percent=1):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Calculate grayscale histogram
    hist = cv2.calcHist([gray],[0],None,[256],[0,256])
    hist_size = len(hist)
    
    # Calculate cumulative distribution from the histogram
    accumulator = []
    accumulator.append(float(hist[0]))
    for index in range(1, hist_size):
        accumulator.append(accumulator[index -1] + float(hist[index]))
    
    # Locate points to clip
    maximum = accumulator[-1]
    clip_hist_percent *= (maximum/100.0)
    clip_hist_percent /= 2.0
    
    # Locate left cut
    minimum_gray = 0
    while accumulator[minimum_gray] < clip_hist_percent:
        minimum_gray += 1
    
    # Locate right cut
    maximum_gray = hist_size -1
    while accumulator[maximum_gray] >= (maximum - clip_hist_percent):
        maximum_gray -= 1
    
    # Calculate alpha and beta values
    alpha = 255 / (maximum_gray - minimum_gray)
    beta = -minimum_gray * alpha

    auto_result = cv2.convertScaleAbs(image, alpha=alpha, beta=beta)
    return (auto_result, alpha, beta)

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

def sendData(fingers):

    string = "$"+str(int(fingers[0]))+str(int(fingers[1]))+str(int(fingers[2]))+str(int(fingers[3]))+str(int(fingers[4]))
    try:
       ser.write(string.encode())
       print(string)
    except:
        pass

def hand_main(img):
    imgResult = img.copy()

    imgBlur = cv2.GaussianBlur(img, (7, 7), 1)
    imgHSV = cv2.cvtColor(imgBlur, cv2.COLOR_BGR2HSV)
    mask = mask_values()
    
    imgMask, imgColorFilter = colorFilter(imgHSV,mask)
    imgOpen =cv2.morphologyEx(imgMask, cv2.MORPH_OPEN,np.ones((5,5),np.uint8))
    imgClosed = cv2.morphologyEx(imgOpen, cv2.MORPH_CLOSE, np.ones((10, 10), np.uint8))
    imgFilter = cv2.bilateralFilter(imgClosed, 5, 75, 75)
    imgContour,imgResult = getContours(imgFilter,imgResult)

    ## TO DISPLAY
    cv2.imshow("Result",imgResult)
    cv2.waitKey(1)

def ros_main():

  rospy.init_node('hand_tracking_py', anonymous=True)

  #initialise the camera
  cap = cv2.VideoCapture(0)
  
  rate = rospy.Rate(10)
  
  while not rospy.is_shutdown():
     
      ret, frame = cap.read()
         
      if ret == True:
      
        auto_result, alpha, beta = automatic_brightness_and_contrast(frame)
      
        hand_main(auto_result)
        
        # Print debugging information to the terminal
        rospy.loginfo('publishing the number of fingers')
             
      # Sleep just enough to maintain the desired rate
      rate.sleep()
 
  # Close down the video stream when done
  cap.release()
  cv2.destroyAllWindows()
  
if __name__ == '__main__':
  ros_main()
