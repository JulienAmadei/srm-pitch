#!/usr/bin/env python3

import rospy # Python library for ROS
from std_msgs.msg import Int16 # Image is the message type
import cv2

################################################################
path = '/home/ubuntu/test_ws/src/camera/src/haarcascades/haarcascade_frontalface_default.xml'  
# PATH OF THE CASCADE
objectName = 'face'       # OBJECT NAME TO DISPLAY
frameWidth = 640
frameHeight = 480

# Colors
GREEN = (0, 255, 0)
RED = (0, 0, 255)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
#################################################################

cv2.namedWindow("Result")
cv2.resizeWindow("Result",frameWidth,frameHeight+100)

# LOAD THE CLASSIFIERS DOWNLOADED
cascade = cv2.CascadeClassifier(path)

# distance from camera to object(face) measured
# centimeter
Known_distance = 43

# width of face in the real world or Object Plane
# centimeter
Known_width = 14

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

def detect_distance(data):
    
    br = CvBridge()
    rospy.loginfo("receiving video frame")
    current_frame = br.imgmsg_to_cv2(data)
    face_width_in_frame = face_data(current_frame)
    Distance = Distance_finder(Focal_length_found, Known_width, face_width_in_frame)
    print("Distance" + str(Distance))
    if (Distance != 'inf'):
        player_detected = True


################
#Initialisation
################7

# reading reference_image from directory
ref_image = cv2.imread("/home/ubuntu/test_ws/src/camera/src/image_refs/face_dist_ref_1.jpg")

# find the face width(pixels) in the reference_image
ref_image_face_width = face_data(ref_image)

# get the focal by calling "Focal_Length_Finder"
# face width in reference(pixels),
# Known_distance(centimeters),
# known_width(centimeters)
Focal_length_found = Focal_Length_Finder(
	Known_distance, Known_width, ref_image_face_width)

player_detected = False
         
def ros_main():
 
  # Tells rospy the name of the node.
  # Anonymous = True makes sure the node has a unique name. Random
  # numbers are added to the end of the name. 
  rospy.init_node('find_player_py', anonymous=True)
  
  # Node is subscribing to the video_frames topic
  rospy.Subscriber('video_frames', Image, detect_distance)
  
  pub = rospy.Publisher('is_player', Image, queue_size=10)
  
  rospy.loginfo('publishing flag for player ')
  
  pub.publish(player_detected)
 
  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()
 
  # Close down the video stream when done
  cv2.destroyAllWindows()
  
if __name__ == '__main__':
  ros_main()
