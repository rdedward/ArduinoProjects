import cv2
import numpy as np

import sys
import serial
import time



cam = cv2.VideoCapture(1)

while True:
  frame = cam.read()[1]

  for i in [1,2,3,4,5,6,7]:
    cv2.circle(frame, (280/7*i,110), 7, (100,0,0), -1)





  cv2.imshow( "Real", frame )


  key = cv2.waitKey(10)
  if key == 27:
    cv2.destroyWindow(winName)
    break

print "Goodbye"

