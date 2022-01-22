#!/usr/bin/env python3

# Import the necessary libraries
import rospy # Python library for ROS
import cv2
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
from cvzone.HandTrackingModule import HandDetector

detector = HandDetector()

def hand_main(data):

    br = CvBridge()
    img = br.imgmsg_to_cv2(data)
    
    # Find the hand and its landmarks
    hands, img = detector.findHands(img)
    if hands:
        hand1 = hands[0]
        lmList1 = hand1["lmList"]  # List of 21 Landmark points
        bbox1 = hand1["bbox"]  # Bounding box info x,y,w,h
        centerPoint1 = hand1['center']  # center of the hand cx,cy
        handType1 = hand1["type"]  # Handtype Left or Right
        
        thumb_edge = lmList1[4]
        
        if thumb_edge[1] > centerPoint1[1] :
            thumb_state = "thumb down"
        elif thumb_edge[1] < centerPoint1[1] :
            thumb_state = "thumb up"

        fingers1 = detector.fingersUp(hand1)
        totalFingers = fingers1.count(1)
        cv2.putText(img, f'Fingers:{totalFingers}', (bbox1[0] + 200, bbox1[1] - 30),
                    cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 2)
        cv2.putText(img,f'{thumb_state}', (30, 30),
                    cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 2)
    
    # Display
    cv2.imshow("Image", img)
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
