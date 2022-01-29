#!/usr/bin/env python3

import rospy # Python library for ROS
import cv2
from camera.srv import *

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

def handle_color_selection(req):
  
  cap = cv2.VideoCapture(0)
  
  rate = rospy.Rate(10)
  
  ret, frame = cap.read()
  
  if ret == True:
      
    auto_result, alpha, beta = automatic_brightness_and_contrast(frame)
    
    imgHsv = cv2.cvtColor(auto_result,cv2.COLOR_BGR2HSV)

    lower = np.array(red[0])
    upper = np.array(red[1])
    mask = cv2.inRange(imgHsv,lower,upper)
    result = cv2.bitwise_and(img,img, mask = mask)

    cv2.imshow('Result', result)
    cv2.imshow('img',img)
    cv2.waitKey(1)
    
  rate.sleep()
 
  # Close down the video stream when done
  cap.release()
  
  return res 

def server_main():
  rospy.init_node('color_selection_server_py')
  s = rospy.Service('color_selection_service', ColorSelectionService, handle_color_selection)
  rospy.spin()

if __name__ == '__main__':
  print("[Camera - Find Player server] Running.")
  
  red = [[-20,0,0],[20,255,255]]
  green = [[100,0,0],[140,255,255]]
  blue = [[220,0,0],[260,255,255]]
  
  server_main()
