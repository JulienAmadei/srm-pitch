#!/usr/bin/env python3

import rospy # Python library for ROS
import cv2
from camera.srv import *
import numpy as np

################################################################

def color_selection_green_client():
    rospy.wait_for_service('color_selection_service')
    try:
        color_selection = rospy.ServiceProxy('color_selection_service', ColorSelectionService)
        res = color_selection([60,0,0,70,255,255])
        return res.height
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def color_selection_blue_client():
    rospy.wait_for_service('color_selection_service')
    try:
        color_selection = rospy.ServiceProxy('color_selection_service', ColorSelectionService)
        res = color_selection([110,0,0,130,255,255])
        return res.height
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def color_selection_red_client():
    rospy.wait_for_service('color_selection_service')
    try:
        color_selection = rospy.ServiceProxy('color_selection_service', ColorSelectionService)
        res = color_selection([0,135,0,5,255,255])
        return res.height
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def handle_wheel(req):
    height_red = color_selection_red_client()
    height_green = color_selection_green_client()
    height_blue = color_selection_blue_client()
    
    print('height_red',height_red)
    print('height_green',height_green)
    print('height_blue',height_blue)
    
    if height_red == height_green == height_blue:
        return "none"
    elif height_red > height_blue: 
        if height_red > height_green:
            return "red"
        else : 
            return "green"
    elif height_green > height_blue:
        return "green"
    else :
        return "blue"
    
def server_main():
  rospy.init_node('wheel_server')
  s = rospy.Service('wheel_service', WheelService, handle_wheel)
  rospy.spin()

if __name__ == '__main__':
  print("[CAMERA PACKAGE] Wheel server running.")  
  server_main()
