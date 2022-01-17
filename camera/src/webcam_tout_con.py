import cv2 # OpenCV library
cap = cv2.VideoCapture(0)

while True :

  ret, frame = cap.read()
  if ret == True:
    cv2.imshow('frame',frame)

