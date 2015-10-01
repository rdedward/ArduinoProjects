import cv2
import numpy as np
import sys
import serial
import time
def treatImg(d1):
  d1 = cv2.medianBlur(d1, 5)
  
  adaptive_thresh_blocksize = 31
  adaptive_thresh = 9 #10 for okay detection
  d1 = cv2.adaptiveThreshold(d1,
                        255,
                        cv2.ADAPTIVE_THRESH_MEAN_C,
                        cv2.THRESH_BINARY_INV,
                        adaptive_thresh_blocksize,
                        adaptive_thresh)
  
  
  kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
  
  d1 = cv2.erode(d1, kernel, iterations = 1)
  d1 = cv2.dilate(d1, kernel, iterations = 12)
  print "shape: ",d1.shape
  return d1

old = []

cam = cv2.VideoCapture(0)


while True:
  frame = cam.read()[1]
  frameM = cv2.medianBlur(frame, 5)
  frameM = cv2.cvtColor(frameM, cv2.COLOR_BGR2HSV)

  (frame1, frame2, frame3) = cv2.split(frameM)


  adaptive_thresh_blocksize = 31
  adaptive_thresh = 40 #10 for okay detection

  frame = cv2.adaptiveThreshold(frame1,
                        255,
                        cv2.ADAPTIVE_THRESH_MEAN_C,
                        cv2.THRESH_BINARY_INV,
                        adaptive_thresh_blocksize,
                        adaptive_thresh)

  cv2.imshow( "Base", frame )

  key = cv2.waitKey(10)
  if key == 27:
    cv2.destroyWindow(winName)
    break

print "Goodbye"

