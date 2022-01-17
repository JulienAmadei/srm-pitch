#!/usr/bin/env python3

import rospy # Python library for ROS
from std_msgs.msg import String
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2

################################################################
path = '/home/lucas/catkin_ws/src/camera/src/haarcascades/haarcascade_frontalface_default.xml'  # PATH OF THE CASCADE
objectName = 'face'       # OBJECT NAME TO DISPLAY
color= (255,0,255)
frameWidth = 640
frameHeight = 480
#################################################################

def empty(a):
    pass

# CREATE TRACKBAR
cv2.namedWindow("Result")
cv2.resizeWindow("Result",frameWidth,frameHeight+100)
cv2.createTrackbar("Scale","Result",200,1000,empty)
cv2.createTrackbar("Neig","Result",8,50,empty)
cv2.createTrackbar("Min Area","Result",3000,100000,empty)

# LOAD THE CLASSIFIERS DOWNLOADED
cascade = cv2.CascadeClassifier(path)

# distance from camera to object(face) measured
# centimeter
Known_distance = 43

# width of face in the real world or Object Plane
# centimeter
Known_width = 14

# Colors
GREEN = (0, 255, 0)
RED = (0, 0, 255)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)

# defining the fonts
fonts = cv2.FONT_HERSHEY_COMPLEX

# focal length finder function
def Focal_Length_Finder(measured_distance, real_width, width_in_rf_image):

	# finding the focal length
	focal_length = (width_in_rf_image * measured_distance) / real_width
	return focal_length

# distance estimation function
def Distance_finder(Focal_Length, real_face_width, face_width_in_frame):

	distance = (real_face_width * Focal_Length)/face_width_in_frame

	# return the distance
	return distance


def face_data(image):

	face_width = 0 # making face width to zero

	# converting color image ot gray scale image
	gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

	# detecting face in the image
	faces = cascade.detectMultiScale(gray_image, 1.3, 5)

	# looping through the faces detect in the image
	# getting coordinates x, y , width and height
	for (x, y, h, w) in faces:

		# draw the rectangle on the face
		cv2.rectangle(image, (x, y), (x+w, y+h), GREEN, 2)

		# getting face width in the pixels
		face_width = w

	# return the face width in pixel
	return face_width




def detect_distance_picture():
    # reading reference_image from directory
    ref_image = cv2.imread("/home/lucas/catkin_ws/src/camera/src/image_refs/face_dist_ref_1.jpg")
    
    cv2.imshow('Original',ref_image)

    # find the face width(pixels) in the reference_image
    ref_image_face_width = face_data(ref_image)

    # get the focal by calling "Focal_Length_Finder"
    # face width in reference(pixels),
    # Known_distance(centimeters),
    # known_width(centimeters)
    Focal_length_found = Focal_Length_Finder(
	Known_distance, Known_width, ref_image_face_width)
    return Focal_length_found
    
def ros_main():
 
  # Node is publishing to the video_frames topic using 
  # the message type Image
  pub = rospy.Publisher('focal_distance', String, queue_size=10)
     
  # Tells rospy the name of the node.
  # Anonymous = True makes sure the node has a unique name. Random
  # numbers are added to the end of the name.
  rospy.init_node('distance_finder_picture', anonymous=True)
     
  # Go through the loop 10 times per second
  rate = rospy.Rate(10) # 10hz
     
  # While ROS is still running.
  while not rospy.is_shutdown():
      
      dist = detect_distance_picture()
         
      # Print debugging information to the terminal
      rospy.loginfo(dist)
             
      # Publish the image.
      # The 'cv2_to_imgmsg' method converts an OpenCV
      # image to a ROS image message
      pub.publish(dist)
             
      # Sleep just enough to maintain the desired rate
      rate.sleep()

if __name__ == '__main__':
  try:
    ros_main()
  except rospy.ROSInterruptException:
    cv2.destroyAllWindows()
    pass
