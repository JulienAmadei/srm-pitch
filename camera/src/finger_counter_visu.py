#!/usr/bin/env python3

# Import the necessary libraries
import rospy # Python library for ROS
import cv2
from std_msgs.msg import Int16 # Image is the message type
from cvzone.HandTrackingModule import HandDetector

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

def hand_main(detector,img):
    
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

  rospy.init_node('finger_counter_sw_py', anonymous=True)
  
  #initialise the hand detector
  detector = HandDetector()

  #initialise the camera
  cap = cv2.VideoCapture(0)
  
  rate = rospy.Rate(10)
  
  while not rospy.is_shutdown():
     
      ret, frame = cap.read()
         
      if ret == True:
      
        auto_result, alpha, beta = automatic_brightness_and_contrast(frame)
      
        hand_main(detector,auto_result)
        
        # Print debugging information to the terminal
        rospy.loginfo('publishing the number of fingers and the state of the thumb')
             
      # Sleep just enough to maintain the desired rate
      rate.sleep()
 
  # Close down the video stream when done
  cap.release()
  cv2.destroyAllWindows()
  
if __name__ == '__main__':
  ros_main()
