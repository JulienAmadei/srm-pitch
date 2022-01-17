#!/usr/bin/env python3

import cv2
import os
import time
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

global countFolder
myPath = '/home/lucas/catkin_ws/src/camera/src/Data/Image' # Rasbperry Pi:  '/home/pi/Desktop/data/images'
cameraNo = 0
moduleVal = 10  # SAVE EVERY ITH FRAME TO AVOID REPETITION
minBlur = 500  # SMALLER VALUE MEANS MORE BLURRINESS PRESENT
grayImage = False # IMAGES SAVED COLORED OR GRAY
saveData = True   # SAVE DATA FLAG
showImage = True  # IMAGE DISPLAY FLAG
imgWidth = 180
imgHeight = 120

br = CvBridge()

def saveDataFunc():
    countFolder = 0
    while os.path.exists( myPath+ str(countFolder)):
        countFolder += 1
    os.makedirs(myPath + str(countFolder))

def create_image_data(data):
    
    count = 0 

    if saveData:saveDataFunc()
    
    current_frame = br.imgmsg_to_cv2(data)
    
    rospy.loginfo('receiving video frame')
    
    #img = cv2.resize(img,(imgWidth,imgHeight))
    if grayImage:current_frame = cv2.cvtColor(current_frame,cv2.COLOR_BGR2GRAY)
    if saveData:
        blur = cv2.Laplacian(current_frame, cv2.CV_64F).var()
        if count % moduleVal ==0 and blur > minBlur:
            nowTime = time.time()
            cv2.imwrite(myPath + str(countFolder) + '/' + str(countSave)+"_"+ str(int(blur))+"_"+str(nowTime)+".png", current_frame)
            countSave+=1
        count += 1
    if showImage:
        cv2.imshow("Image", current_frame)
        
def ros_main():
 
  # Tells rospy the name of the node.
  # Anonymous = True makes sure the node has a unique name. Random
  # numbers are added to the end of the name. 
  rospy.init_node('video_sub_py', anonymous=True)
   
  # Node is subscribing to the video_frames topic
  rospy.Subscriber('video_frames', Image, create_image_data)
 
  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()
 
  # Close down the video stream when done
  cv2.destroyAllWindows()
  cv2.release()
  
if __name__ == '__main__':
  ros_main()
