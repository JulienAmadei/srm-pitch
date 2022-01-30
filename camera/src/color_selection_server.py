#!/usr/bin/env python3

import rospy # Python library for ROS
import cv2
from camera.srv import *
import numpy as np

################################################################

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

def colorFilter(img, vals):
    lower = np.array([vals[0],vals[1],vals[2]])
    upper = np.array([vals[3],vals[4],vals[5]])
    mask = cv2.inRange(img, lower, upper)
    imgColorFilter = cv2.bitwise_and(img, img, mask=mask)
    ret, imgMask = cv2.threshold(mask, 127, 255, 0)
    return imgMask,imgColorFilter

def getContours(imgCon,imgMatch):
    contours, hierarchy = cv2.findContours(imgCon, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    imgCon = cv2.cvtColor(imgCon,cv2.COLOR_GRAY2BGR)
    bigCon = 0
    myCounter=0
    myPos = np.zeros(4)
    cY = 0
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if (area>15000):
            M = cv2.moments(cnt)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
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
    return bigCon, cY

def handle_color_selection(img,values):
  
    imgResult = img.copy()

    imgBlur = cv2.GaussianBlur(img, (7, 7), 1)
    imgHSV = cv2.cvtColor(imgBlur, cv2.COLOR_BGR2HSV)
    
    imgMask, imgColorFilter = colorFilter(imgHSV,values)
    imgOpen =cv2.morphologyEx(imgMask, cv2.MORPH_OPEN,np.ones((5,5),np.uint8))
    imgClosed = cv2.morphologyEx(imgOpen, cv2.MORPH_CLOSE, np.ones((10, 10), np.uint8))
    imgFilter = cv2.bilateralFilter(imgClosed, 5, 75, 75)
    contours, height = getContours(imgFilter,imgResult)
    
    return contours, height

def color_main(req):
  
  #initialise the camera
  cap = cv2.VideoCapture(0)
  
  rate = rospy.Rate(10)
  
  occurrence = False
  
  t = 0
  
  while not occurrence and t < 20:
      
      t += 1
     
      ret, frame = cap.read()
         
      if ret == True:
      
        auto_result, alpha, beta = automatic_brightness_and_contrast(frame)
        
        contours, height = handle_color_selection(auto_result,req.color_code)
      
        if contours > 0 :
          
          occurrence = True
             
      # Sleep just enough to maintain the desired rate
      rate.sleep()
 
  # Close down the video stream when done
  cap.release()
  
  return occurrence, height

def server_main():
  rospy.init_node('color_selection_server_py')
  s = rospy.Service('color_selection_service', ColorSelectionService, color_main)
  rospy.spin()

if __name__ == '__main__':
  print("[CAMERA PACKAGE] Color selection server running.")  
  server_main()
