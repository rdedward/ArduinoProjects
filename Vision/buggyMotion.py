import cv2
import numpy as np
import sys
import serial
import time

def diffImg(t0, t1, t2):
  d1 = cv2.absdiff(t2, t1)
  d2 = cv2.absdiff(t1, t0)
  return cv2.bitwise_and(d1, d2)


def diffImg2(t0, t2):
  #print "t0 shape: ", t0.shape
  d1 = cv2.absdiff(t2, t0)
  #print "d1 shape: ", d1.shape
  return d1

def vis_to_ard(locx,locy):
    servo_midy = 95 #neutral y and x 
    servo_midx = 95

    servo_miny = 70
    servo_maxy = 115

    servo_minx = 70
    servo_maxx = 130


    locx = locx-320/2 #difference from center
    locy = locy-240/2

    width = 130-70
    height = 115-70
    newx = width/2*locx/(320/2)+servo_midx #percent of difference from center*center
    newy = height/2*locy/(240/2)+servo_midy


    if(newx < servo_minx):
        newx = servo_minx
        print "exceeded minx"


    if(newy < servo_miny):
        newy = servo_miny
        print "exceeded miny"

    if(newx > servo_maxx):
        newx = servo_maxx
        print "exceeded maxx"

    if(newy > servo_maxy):
        newy = servo_maxy
        print "exceeded maxy"
    
    sendValue(newx,newy)


def sendValue(x,y):
    ser = serial.Serial('/dev/ttyACM1',9600)
    ser.write(str(int(x))+'\n'+str(int(y))+'\n')
    print "sent value"


def detectBlobs(d1):
  contours, hierarchy = cv2.findContours(d1,
                                         cv2.RETR_EXTERNAL,
                                         cv2.CHAIN_APPROX_SIMPLE)

  raw_bins = []

  if len(contours) > 1:
      cnt = contours[0]
      cv2.drawContours(d1, contours, -1, (255, 255, 255), 3)
      max_w = 0
      max_h = 0
      max_x = 0
      max_y = 0
      avg_x = 0
      avg_y = 0
      count = 0
      for h, cnt in enumerate(contours):
          #hull = cv2.convexHull(cnt)
          rect = cv2.minAreaRect(cnt)
          box = cv2.cv.BoxPoints(rect)
          box = np.int0(box)

          x,y,w,h = cv2.boundingRect(cnt)
          avg_x += x+w/2
          avg_y += y+h/2
          count += 1
          if (w*h >max_w*max_h):
              max_w, max_h, max_x, max_y = w, h, x, y
      d1 = cv2.cvtColor(d1, cv2.COLOR_GRAY2BGR,d1, 3)
      cv2.rectangle(d1,(max_x,max_y),(max_x+max_w,max_y+max_h),(0,255,0),2)
      cv2.circle(d1, (int(avg_x/count), int(avg_y/count)), 15, (255,0,0), -1)
      cv2.circle(d1, (int(max_x+max_w/2), int(max_y+max_h/2)), 15, (0,255,0), -1) #good
      vis_to_ard(int(max_x+max_w/2),int(max_y+max_h/2))
      
  return d1

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


cam = cv2.VideoCapture(1)

winName = "Movement Indicator"
cv2.namedWindow(winName, cv2.CV_WINDOW_AUTOSIZE)

# Read three images first:
t_minus = cv2.cvtColor(cam.read()[1], cv2.COLOR_RGB2GRAY)
t = cv2.cvtColor(cam.read()[1], cv2.COLOR_RGB2GRAY)
t_plus = cv2.cvtColor(cam.read()[1], cv2.COLOR_RGB2GRAY)

while len(old)< 20:
  template = cv2.cvtColor(cam.read()[1], cv2.COLOR_RGB2GRAY)
  old.append(template)

while True:
  #cv2.imshow( winName, diffImg2(t_minus, t_plus) )
  cv2.imshow( winName, detectBlobs(treatImg(diffImg2(t_minus, t_plus))) )
  #cv2.imshow( winName, treatImg(diffImg2(t_minus, t_plus)) )

  # Read next image
  t_minus = old[0]
  t = old[10]
  t_plus = cv2.cvtColor(cam.read()[1], cv2.COLOR_RGB2GRAY)
  old.append(t_plus)
  old = old[:]
  print len(old)
  key = cv2.waitKey(10)
  if key == 27:
    cv2.destroyWindow(winName)
    break

print "Goodbye"

