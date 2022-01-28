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

def face_data(image, cascade):

	face_width = 0 # making face width to zero

	# converting color image ot gray scale image
	gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

	# detecting face in the image
	faces = cascade.detectMultiScale(gray_image, 1.3, 5)

	# looping through the faces detect in the image
	# getting coordinates x, y , width and height
	for (x, y, h, w) in faces:

		# getting face width in the pixels
		face_width = w
		
	# return the face width in pixel
	return face_width

def handle_find_player(req):
 
  #initialise the camera
  cap = cv2.VideoCapture(0)
  
  rate = rospy.Rate(10)
  
  #reference image  (CHANGER LES PATHS EN FONCTION DE L'ORDI OU DU PITCH)
  ref_image = cv2.imread("/home/lucas/catkin_ws/src/camera/src/image_refs/face_dist_ref_1.jpg")

  path = '/home/lucas/catkin_ws/src/camera/src/haarcascades/haarcascade_frontalface_default.xml'
  
  # LOAD THE CLASSIFIERS DOWNLOADED
  cascade = cv2.CascadeClassifier(path)
  
  # distance from camera to object(face) measured
  Known_distance = 43

  # width of face in the real world or Object Plane
  known_width = 14
  
  # find the face width(pixels) in the reference_image
  ref_image_face_width = face_data(ref_image, cascade)
  
  # get the focal
  focal_length_found = (ref_image_face_width * Known_distance) / known_width
  
  distance = -1
  
  while distance == -1 or distance > 100 :
     
      ret, frame = cap.read()
         
      if ret == True:
      
        auto_result, alpha, beta = automatic_brightness_and_contrast(frame)
      
        face_width_in_frame = face_data(auto_result,cascade)
    
        if face_width_in_frame == 0 :
        
          distance = -1
          
        else :
            
          distance = (known_width * focal_length_found)/face_width_in_frame
        
        print(distance)
             
      # Sleep just enough to maintain the desired rate
      rate.sleep()
 
  # Close down the video stream when done
  cap.release()
  
  return int(distance)

def server_main():
  rospy.init_node('find_player_server_py')
  s = rospy.Service('find_player_service', FindPlayerService, handle_find_player)
  rospy.spin()

if __name__ == '__main__':
  server_main()
