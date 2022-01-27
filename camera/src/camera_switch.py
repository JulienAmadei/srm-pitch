#!/usr/bin/env python3

import rospy
from CameraSwitch.srv import *

def find_player_client():
    rospy.wait_for_service('find_player_service')
    try:
        find_player = rospy.ServiceProxy('find_player_service', FindPlayerService)
        res = find_player()
        return res.distance
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def finger_counter_client():
    rospy.wait_for_service('finger_counter_service')
    try:
        finger_counter = rospy.ServiceProxy('finger_counter_service', FingerCounterService)
        res = finger_counter()
        return res.nb_finger, res.thumb_state
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def color_recognition_client():
    rospy.wait_for_service('color_recognition_service')
    try:
        color_recognition = rospy.ServiceProxy('color_recognition_service', ColorRecognitionService)
        res = color_recognition()
        return res.color_code
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def handle_switch(req):
    match req.obj :
    
        case 'hand':
            return finger_counter_client()
        
        case 'face':
            return find_player_client()
        
        case 'color':
            return color_recognition_client()

def switch_server():
    rospy.init_node('camera_switch_py')
    s = rospy.Service('camera_service', CameraService, handle_switch)
    rospy.spin()

if __name__ == "__main__":
    switch_server()
    
