#!/usr/bin/env python3

import rospy
from camera.srv import SwitchService


def camera_client(data):
    rospy.wait_for_service('camera_service')
    try:
        switch = rospy.ServiceProxy('camera_service', SwitchService)
        res = switch(data)
        return res.nb_finger, res.thumb_state, res.distance
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)



if __name__ == "__main__":
    obj_to_detect = 'hand'
    var1 = camera_client(obj_to_detect)
    nb_finger = var1[0]
    thumb_state = var1[1]
    obj_to_detect = 'face'
    var2 = camera_client(obj_to_detect)
    distance = var2[2]
    print(nb_finger,thumb_state,distance)
