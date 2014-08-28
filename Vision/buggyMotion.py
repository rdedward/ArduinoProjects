import cv2
import numpy as np

def diffImg(t0, t1, t2):
  d1 = cv2.absdiff(t2, t1)
  d2 = cv2.absdiff(t1, t0)
  return cv2.bitwise_and(d1, d2)


def diffImg2(t0, t2):
  d1 = cv2.absdiff(t2, t0)
  return d1


def detectBlobs(d1):
  contours, hierarchy = cv2.findContours(d1,
                                         cv2.RETR_EXTERNAL,
                                         cv2.CHAIN_APPROX_SIMPLE)

  raw_bins = []

  if len(contours) > 1:
      cnt = contours[0]
      cv2.drawContours(d1, contours, -1, (255, 255, 255), 3)

      for h, cnt in enumerate(contours):
          #hull = cv2.convexHull(cnt)
          rect = cv2.minAreaRect(cnt)
          box = cv2.cv.BoxPoints(rect)
          box = np.int0(box)
  return d1

def treatImg(d1):
  d1 = cv2.medianBlur(d1, 5)
  
  adaptive_thresh_blocksize = 25
  adaptive_thresh = 10 #10 for okay detection
  d1 = cv2.adaptiveThreshold(d1,
                        255,
                        cv2.ADAPTIVE_THRESH_MEAN_C,
                        cv2.THRESH_BINARY_INV,
                        adaptive_thresh_blocksize,
                        adaptive_thresh)
  

  kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
  d1 = cv2.erode(d1, kernel, iterations = 1)
  d1 = cv2.dilate(d1, kernel, iterations = 1)
  print d1.shape
  return d1

old = []


cam = cv2.VideoCapture(0)


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
  #cv2.imshow( winName, diffImg(t_minus, t, t_plus) )
  cv2.imshow( winName, treatImg(diffImg2(t_minus, t_plus)) )
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
