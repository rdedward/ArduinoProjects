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
  winNameD = "Diff Frame"
  cv2.namedWindow(winNameD, cv2.CV_WINDOW_AUTOSIZE)
  cv2.imshow(winNameD, d1)
  print "size is: ", d1.shape
  return d1

def vis_to_ard(locx,locy):
    servo_midy = 90 #neutral y and x 
    servo_midx = 100

    servo_miny = 70
    servo_maxy = 115

    servo_minx = 68
    servo_maxx = 132


    locx = locx-320/2 #difference from center
    locy = locy-240/2

    width = servo_maxx-servo_minx
    height = servo_maxy-servo_miny
    newx = width/2*locx/(320/2)+servo_midx #percent of difference from center*center
    newy = height/2*locy/(240/2)+servo_midy

    newy += 7

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
    
    sendValue(newx, newy)


def sendValue(x,y):
    try:
      ser = serial.Serial('/dev/ttyACM1',9600)
    except serial.SerialException:
      print "caught exception"
      ser = serial.Serial('/dev/ttyACM0',9600)
    ser.write(str(int(x)*10)+'\n'+str(int(y)*10+1)+'\n')
    print "sent value:", int(x)*10, ":", int(y)*10+1

def sendValueS(x):
    try:
      ser = serial.Serial('/dev/ttyACM1',9600)
    except serial.SerialException:
      print "caught exception"
      ser = serial.Serial('/dev/ttyACM0',9600)
    ser.write(str(int(x))+'\n')
    print "sent value:", int(x)

def readSerial():
    print "SerialB"
    ser = serial.Serial('/dev/ttyACM0',9600)
    lines = ser.readline()
    print lines
    print "Serial End"

def detectBlobs(d1, fU, lastseen, frameN):
  contours, hierarchy = cv2.findContours(d1,
                                         cv2.RETR_EXTERNAL,
                                         cv2.CHAIN_APPROX_SIMPLE)

  raw_bins = []

  if len(contours) > 0:
      print "contours found"
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
      found = 0
      if max_w*max_h > 1000:
        lastseen = 20
        last_w, last_h, last_x, last_y = max_w, max_h, max_x, max_y
        found = 1
      elif lastseen > 0:
        lastseen -= 1
        max_w, max_h, max_x, max_y = last_w, last_h, last_x, last_y
      if found == 1:        
        #sendValueS(11111);
        crop_f = frameU[max_y:max_y+max_h, max_x:max_x+max_w]
        print crop_f.size
        if crop_f.size > 4200:
          cv2.imshow("cropped", crop_f)
          cv2.imwrite("./Images/img"+str(frameN)+".jpg", crop_f)
          cv2.circle(d1, (int(max_x+max_w/2), int(max_y+max_h/2)), 15, (0,255,0), -1) #good
          cv2.circle(fU, (int(max_x+max_w/2), int(max_y+max_h/2)), 15, (0,255,0), -1) #good
          print "found"
      else:
        #sendValueS(00000);
        cv2.destroyWindow("cropped")
      vis_to_ard(int(max_x+max_w/2),int(max_y+max_h/2))
      
  return d1

def treatImg(d1):
  d1 = cv2.medianBlur(d1, 5)
  
  adaptive_thresh_blocksize = 7
  adaptive_thresh = 30 #10 for okay detection
  '''
  d1 = cv2.adaptiveThreshold(d1,
                        255,
                        cv2.ADAPTIVE_THRESH_MEAN_C,
                        cv2.THRESH_BINARY_INV,
                        adaptive_thresh_blocksize,
                        adaptive_thresh)
  '''

  ret, d1 = cv2.threshold(d1, adaptive_thresh, 255, cv2.THRESH_BINARY)
  
  
  kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
  
  d1 = cv2.erode(d1, kernel, iterations = 1)
  d1 = cv2.dilate(d1, kernel, iterations = 4)
  #print "shape: ",d1.shape
  winNameA = "Adaptive Frame"
  cv2.namedWindow(winNameA, cv2.CV_WINDOW_AUTOSIZE)
  cv2.imshow(winNameA, d1)
  return d1

old = []


cam = cv2.VideoCapture(1)

winName = "Movement Indicator"
cv2.namedWindow(winName, cv2.CV_WINDOW_AUTOSIZE)

winName2 = "Unchanged Frame"
cv2.namedWindow(winName2, cv2.CV_WINDOW_AUTOSIZE)

# Read three images first:
t_minus = cv2.cvtColor(cam.read()[1], cv2.COLOR_RGB2GRAY)
t = cv2.cvtColor(cam.read()[1], cv2.COLOR_RGB2GRAY)
t_plus = cv2.cvtColor(cam.read()[1], cv2.COLOR_RGB2GRAY)

lastseen = 0

while len(old)< 20:
  template = cv2.cvtColor(cam.read()[1], cv2.COLOR_RGB2GRAY)
  old.append(template)

last_w, last_h, last_x, last_y = None, None, None, None
while True:
  frameU = cam.read()[1]
  frameC = frameU.copy()
  cv2.imshow( winName, detectBlobs(treatImg(diffImg2(t_minus, t_plus)), frameU, lastseen, len(old)) )

  # Read next image
  t_minus = old[0]
  t = old[10]
  #frameU = cam.read()[1]
  cv2.imshow( winName2, frameU)
  t_plus = cv2.cvtColor(frameU, cv2.COLOR_RGB2GRAY)
  old.append(t_plus)
  old = old[:]
  print "Frame: ", len(old)
  key = cv2.waitKey(10)
  if key == 27:
    sendValue(90,90)
    cv2.destroyWindow(winName)
    break

print "Goodbye"

