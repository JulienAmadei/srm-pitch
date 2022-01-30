#!/usr/bin/env python3

import rospy
from camera.srv import *

def find_player_client():
    rospy.wait_for_service('find_player_service')
    try:
        find_player = rospy.ServiceProxy('find_player_service', FindPlayerService)
        res = find_player()
        return 0, 0, res.distance, 'none', 0, 0
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def finger_counter_client():
    rospy.wait_for_service('finger_counter_service')
    try:
        finger_counter = rospy.ServiceProxy('finger_counter_service', FingerCounterService)
        res = finger_counter()
        return res.nb_finger, res.thumb_state, 0, 'none', 0, 0
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def wheel_client():
    rospy.wait_for_service('wheel_service')
    try:
        wheel_color = rospy.ServiceProxy('wheel_service', WheelService)
        res = wheel_color()
        return 0, 0, 0, res.color_name, 0, 0
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def color_selection_green_client():
    rospy.wait_for_service('color_selection_service')
    try:
        color_selection = rospy.ServiceProxy('color_selection_service', ColorSelectionService)
        res = color_selection([70,0,0,90,255,255])
        return 0, 0, 0, 'none', res.occurrence, 0
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def color_selection_blue_client():
    rospy.wait_for_service('color_selection_service')
    try:
        color_selection = rospy.ServiceProxy('color_selection_service', ColorSelectionService)
        res = color_selection([95,0,0,140,255,255])
        return 0, 0, 0, 'none', res.occurrence, 0
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def color_selection_red_client():
    rospy.wait_for_service('color_selection_service')
    try:
        color_selection = rospy.ServiceProxy('color_selection_service', ColorSelectionService)
        res = color_selection([140,0,0,180,255,255])
        return 0, 0, 0, 'none', res.occurrence, 0
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def handle_switch(req):
    switch =  {
        "hand": finger_counter_client,			#return number of fingers and the thumb state
        "face": find_player_client,				#return distance
        "red": color_selection_red_client, 			#return bool
        "green": color_selection_green_client, 		#return bool
        "blue": color_selection_blue_client, 			#return bool
        "wheel": wheel_client, 				#return color_code or [0,0,0] if wheel is not found
    }
    
    chosen_client = switch.get(req.obj)
    
    return chosen_client()

def switch_server():
    rospy.init_node('camera_switch_py')
    s = rospy.Service('camera_service', SwitchService, handle_switch)
    rospy.spin()

if __name__ == "__main__":
    print("[Camera - Switcher server] Running.")
    switch_server()
    
