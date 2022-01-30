#!/usr/bin/env python3

import rospy
from camera.srv import SwitchService


def camera_client(data):
    rospy.wait_for_service('camera_service')
    try:
        switch = rospy.ServiceProxy('camera_service', SwitchService)
        res = switch(data)
        return res.nb_finger, res.thumb_state, res.distance, res.color_code, res.color_occurrence
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)



if __name__ == "__main__":
    obj_to_detect = 'wheel'
    var1 = camera_client(obj_to_detect)
    print(var1)
    rospy.spin()
