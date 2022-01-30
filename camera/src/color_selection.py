#!/usr/bin/env python3
 
# Import the necessary libraries
import rospy # Python library for ROS
import cv2 # OpenCV library
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import numpy as np


def empty(a):
    pass

cv2.namedWindow("HSV")
cv2.createTrackbar("HUE Min","HSV",0,179,empty)
cv2.createTrackbar("HUE Max","HSV",179,179,empty)
cv2.createTrackbar("SAT Min","HSV",0,255,empty)
cv2.createTrackbar("SAT Max","HSV",255,255,empty)
cv2.createTrackbar("VALUE Min","HSV",0,255,empty)
cv2.createTrackbar("VALUE Max","HSV",255,255,empty)

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
    h_min = cv2.getTrackbarPos("HUE Min","HSV")
    h_max = cv2.getTrackbarPos("HUE Max", "HSV")
    s_min = cv2.getTrackbarPos("SAT Min", "HSV")
    s_max = cv2.getTrackbarPos("SAT Max", "HSV")
    v_min = cv2.getTrackbarPos("VALUE Min", "HSV")
    v_max = cv2.getTrackbarPos("VALUE Max", "HSV")
    
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
        if (area>10000):
            M = cv2.moments(cnt)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            cv2.circle(imgMatch, (cX, cY), 7, (255, 255, 255), -1)
            cv2.putText(imgMatch, "center", (cX - 20, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
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
    return imgCon,imgMatch

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
    stack = np.hstack([imgColorFilter,imgContour,imgResult])
    cv2.imshow("stack",stack)
    cv2.imshow("Result",imgResult)
    cv2.waitKey(1)

def color_main():
  
  rospy.init_node('color_selection_py', anonymous=True)
  
  #initialise the camera
  cap = cv2.VideoCapture(0)
  
  rate = rospy.Rate(10)
  
  while not rospy.is_shutdown():
     
      ret, frame = cap.read()
         
      if ret == True:
      
        auto_result, alpha, beta = automatic_brightness_and_contrast(frame)
      
        hand_main(auto_result)
             
      # Sleep just enough to maintain the desired rate
      rate.sleep()
 
  # Close down the video stream when done
  cap.release()
  cv2.destroyAllWindows()
  
if __name__ == '__main__':
  color_main()
