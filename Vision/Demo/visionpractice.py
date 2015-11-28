import cv2
import numpy as np
import math

import cv

def draw_linesC(src, lines, color, limit=None):
    '''Draws lines on the image in given color, because I'm sick of red.

    Arguments:
        src - Source image which to draw lines on.
        lines - A list of lines.  This should be in the format of rho, theta
            pairs: [(rho, theta), ...].  This is the same format which
            cv.HoughLines2() returns.
        limit - If given, it only that many number of lines will be drawn.

    '''

    if not limit:
        limit = len(lines)

    for rho, theta in lines[:limit]:
        a = math.cos(theta)
        b = math.sin(theta)
        x0 = a * rho
        y0 = b * rho
        pt1 = (cv.Round(x0 + 1000 * (-b)), cv.Round(y0 + 1000 * (a)))
        pt2 = (cv.Round(x0 - 1000 * (-b)), cv.Round(y0 - 1000 * (a)))
        cv.Line(src, pt1, pt2, cv.RGB(color[0], color[1], color[2]), 3, cv.CV_AA, 0)



def cv_to_cv2(frame):
    '''Convert a cv image into cv2 format.

    Keyword Arguments:
    frame -- a cv image
    Returns a numpy array that can be used by cv2.
    '''
    cv2_image = np.asarray(frame[:, :])
    return cv2_image


def cv2_to_cv(frame):
    '''Convert a cv2 image into cv format.

    Keyword Arguments:
    frame -- a cv2 numpy array representing an image.
    Returns a cv image.
    '''
    container = cv.fromarray(frame)
    cv_image = cv.GetImage(container)
    return cv_image




old = []

cam = cv2.VideoCapture(0)


while True:
  frame = cam.read()[1]
  frame = cv2.medianBlur(frame, 5)
  frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

  (frame1, frame2, frame3) = cv2.split(frameHSV)


  adaptive_thresh_blocksize = 31
  adaptive_thresh = 35 #10 for okay detection

  
  frameA = cv2.adaptiveThreshold(frame2,
                        255,
                        cv2.ADAPTIVE_THRESH_MEAN_C,
                        cv2.THRESH_BINARY_INV,
                        adaptive_thresh_blocksize,
                        adaptive_thresh)

  kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
  
  frameA = cv2.erode(frameA, kernel, iterations = 1)
  frameA = cv2.dilate(frameA, kernel, iterations = 1)


  # Get Edges
  #frameA = cv2.Canny(frameA, 30, 40)

  
  raw_linesG = cv2.HoughLinesP(frameA,
                                    rho=1,
                                    theta=math.pi / 180,
                                    threshold=100,
                                    minLineLength = 150,
                                    maxLineGap = 20)
  frame = cv2_to_cv(frame)
  '''
  vert_lines = []
  if raw_linesG is not None:
    for line1 in raw_linesG:

      if line1[0] > 0 and line1[0] < pi/4: 
        vert_lines.append(line1)
      if line1[0] > 3*pi/4 and line1[0] < pi:
        vert_lines.append(line1)
  '''



  if raw_linesG is not None:
    print raw_linesG[0]
    print "Bacon"
    for line in raw_linesG:
      cv.Line(frame, (int(line[0]),int(line[1])), (int(line[2]),int(line[3])), (200, 140, 2), 10, cv.CV_AA, 0)



  frame = cv_to_cv2(frame)


  cv2.imshow( "Base", frameA )
  cv2.imshow( "Base2", frame  )

  key = cv2.waitKey(10)
  if key == 27:
    cv2.destroyWindow(winName)
    break

print "Goodbye"

