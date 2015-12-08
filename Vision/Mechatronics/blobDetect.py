import cv2
import numpy as np
import math
import serial

def vis_to_ard(locx,locy):
    servo_midy = 90 #neutral y and x 
    servo_midx = 100

    servo_miny = 70
    servo_maxy = 115

    servo_minx = 68
    servo_maxx = 132


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
    try:
      ser = serial.Serial('/dev/ttyACM1',9600)
    except serial.SerialException:
      print "caught exception"
      ser = serial.Serial('/dev/ttyACM0',9600)
    ser.write(str(int(x)*10)+'\n'+str(int(y)*10+1)+'\n')
    print "sent value:", int(x)*10, ":", int(y)*10+1


def determineMain(confirmed, frame):
  max_x, max_y, max_area = 0, 0, 0
  for bin in confirmed:
    if max_x == 0 and max_y == 0:
      max_x = bin.midx
      max_y = bin.midy
      continue
    if bin.area > max_area:
      max_x = bin.midx
      max_y = bin.midy
      max_area = bin.area
  cv2.circle(frame, (int(max_x), int(max_y)), 4 , (125,255,0), -1) #good
  vis_to_ard(max_x, max_y)

#############################################################################
#############################################################################
class Bin():
    bin_id = 0
    """ an imaged bin b """

    def __init__(self, corner1, corner2, corner3, corner4):
        self.type = "bin"
        self.corner1 = corner1
        self.corner2 = corner2
        self.corner3 = corner3
        self.corner4 = corner4
        self.corners = [corner1, corner2, corner3, corner4]
        #self.theta = 0
        self.midx = rect_midpointx(corner1, corner2, corner3, corner4)
        self.midy = rect_midpointy(corner1, corner2, corner3, corner4)
        self.area = 0
        self.id = 0        # id identifies which bin your looking at
        self.lastseen = 1  # how recently you have seen this bin
        # how many times you have seen this bin (if you see it enough it
        # becomes confirmed)
        self.seencount = 0



        dy = (self.corner2[1] - self.corner1[1])
        dx = (self.corner2[0] - self.corner1[0])
        self.line_slope = dy / dx
        
        size1 = line_distance(corner1,corner2)
        size2 = line_distance(corner1,corner4)
        if size1 < size2:
            self.width = size1
            self.height = size2
            #self.theta = angle_between_lines(corner1,corner3,corner2)
            #self.theta = line_angle(corner1,corner2)
        else:
            self.width = size2
            self.height = size1
            #self.theta = angle_between_lines(corner1,corner2,corner3)
            #self.theta = line_angle(corner1,corner4)



def line_distance(corner_a, corner_b):
    distance = math.sqrt((corner_b[0] - corner_a[0]) ** 2 +
                         (corner_b[1] - corner_a[1]) ** 2)
    return distance


def rect_midpointx(corner_a, corner_b, corner_c, corner_d):
    midpoint_x = (corner_a[0] + corner_b[0] + corner_c[0] + corner_d[0]) / 4
    return midpoint_x


def rect_midpointy(corner_a, corner_b, corner_c, corner_d):
    midpoint_y = (corner_a[1] + corner_b[1] + corner_c[1] + corner_d[1]) / 4
    return midpoint_y


def angle_between_lines(corner_a, corner_b, corner_c):
    slope_a = line_slope(corner_a, corner_b)
    slope_b = line_slope(corner_a, corner_c)

    if slope_a is not None and slope_b is not None and (1 + slope_a * slope_b) != 0:
        angle = math.atan((slope_a - slope_b) / (1 + slope_a * slope_b))
        return angle
    else:
        angle = 0
        return angle

def draw_bins(binlist, frame):
  ind_bins = []
  clr = (0, 0, 255)
  for bin in binlist:
    cv2.circle(frame,
               bin.corner1, 5, clr, -1)
    cv2.circle(frame,
               bin.corner2, 5, clr, -1)
    cv2.circle(frame,
               bin.corner3, 5, clr, -1)
    cv2.circle(frame,
               bin.corner4, 5, clr, -1)
    cv2.circle(frame, (int(bin.midx), int(bin.midy)), 5, clr, -1)

    #cv2.rectangle(self.debug_frame, bin.corner1, bin.corner3, clr, 5)
    pts = np.array([bin.corner1, bin.corner2, bin.corner3, bin.corner4], np.int32)
    pts = pts.reshape((-1, 1, 2))
    cv2.polylines(frame, [pts], True, (255, 0, 0), 4)

    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(frame, "id=" + str(bin.id) + "lastseen:" + str(bin.lastseen), (int(bin.midx) - 50, int(bin.midy) + 40), font, .3, clr, 1, cv2.CV_AA)

    #cv2.putText(frame, "corner1", (int(bin.corner1[0]), int(bin.corner1[1])), font, .3, clr, 1, cv2.CV_AA)
    '''
            # draw angle line
            m = math.tan(bin.theta)
            pt1 = (int(bin.midx), int(bin.midy))
            pt2 = (int(bin.midx + 10), int((10) * m + bin.midy))
            cv2.line(self.debug_frame, pt1, pt2, clr, 4, 8, 0)
    '''





#############################################################################
#############################################################################

cam = cv2.VideoCapture(1)

#Adaptive Thresholds
maxVal = 50
blocksize = 11
#BLob Thresholds
min_area = 100#4500
max_area = 1000000000#14000
#Group Threshold
deflection = 50

lastseen_cap = 20

id_num = 0

'''
Creates Windows
'''
winName1 = "Unmodified"
cv2.namedWindow(winName1, cv2.CV_WINDOW_AUTOSIZE)

winName2 = "Gray"
cv2.namedWindow(winName2, cv2.CV_WINDOW_AUTOSIZE)

winName3 = "Adaptive"
cv2.namedWindow(winName3, cv2.CV_WINDOW_AUTOSIZE)

winName4 = "Contours"
cv2.namedWindow(winName4, cv2.CV_WINDOW_AUTOSIZE)

winName5 = "raw Contours"
cv2.namedWindow(winName5, cv2.CV_WINDOW_AUTOSIZE)

confirmed = []

while True:
  frame = cam.read()[1]
  cv2.imshow( winName1, frame )

  hsv_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

  (hue, sat, value) = cv2.split(hsv_frame)
  chosen = value

  cv2.imshow(winName2, chosen)


  ret, adaptive = cv2.threshold(chosen, maxVal, 255, cv2.THRESH_BINARY_INV)

  kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
  adaptive = cv2.erode(adaptive, kernel)
  adaptive = cv2.dilate(adaptive, kernel)
  #adaptive = cv2.dilate(adaptive, kernel)
  #adaptive = cv2.dilate(adaptive, kernel)

  cv2.imshow(winName3, adaptive)

  ######## FINDING CONTOURS #######
  # Find contours
  contours, hierarchy = cv2.findContours(adaptive,
                                         cv2.RETR_EXTERNAL,
                                         cv2.CHAIN_APPROX_SIMPLE)

  raw_blobs = []

  if len(contours) > 1:
    cnt = contours[0]
    cv2.drawContours(adaptive, contours, -1, (255, 255, 255), 3)

    for h, cnt in enumerate(contours):
      rect = cv2.minAreaRect(cnt)
      box = cv2.cv.BoxPoints(rect)
      box = np.int0(box)

      # test aspect ratio & area, create bin if matches
      (x, y), (w, h), theta = rect
      if w > 0 and h > 0:
        area = h * w
        if min_area < area < max_area:
          approx = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)
          aspect_ratio = float(h) / w
          # Depending on the orientation of the bin, "width" may be flipped with height, thus needs 2 conditions for each case
          if (0.8 < aspect_ratio < 1.2):
            new_bin = Bin(tuple(box[0]), tuple(
            box[1]), tuple(box[2]), tuple(box[3]))
            new_bin.id = id_num
            new_bin.area = area
            id_num += 1
            new_bin.area = area
            raw_blobs.append(new_bin)

  for blob1 in raw_blobs:
    found = 0
    for blob2 in confirmed:
      if(math.fabs(blob1.midx - blob2.midx) < deflection and 
         math.fabs(blob1.midy - blob2.midy) < deflection ):
        #blob2 = blob1
        blob2.corner1 = blob1.corner1
        blob2.corner2 = blob1.corner2
        blob2.corner3 = blob1.corner3
        blob2.corner4 = blob1.corner4
        blob2.corners = [blob1.corner1, blob1.corner2, blob1.corner3, blob1.corner4]
        #self.theta = 0
        blob2.midx = rect_midpointx(blob1.corner1, blob1.corner2, blob1.corner3, blob1.corner4)
        blob2.midy = rect_midpointy(blob1.corner1, blob1.corner2, blob1.corner3, blob1.corner4)
        found = 1
        blob2.lastseen += 2
        if(blob2.lastseen > lastseen_cap):
          blob2.lastseen = lastseen_cap
        
    if found == 0:
      blob1.lastseen += 1
      confirmed.append(blob1)
      #print "confirmed appended"
        
  for blob in confirmed:
    if blob.lastseen < 0:
      confirmed.remove(blob)
    blob.lastseen -= 1

  conf_contour_frame = frame.copy()
  raw_contour_frame = conf_contour_frame.copy()
  #print "num: "+ str(len(confirmed))
  
  draw_bins(confirmed, conf_contour_frame)
  draw_bins(raw_blobs, raw_contour_frame)
  
  #determineMain(confirmed, conf_contour_frame)

  cv2.imshow(winName4, conf_contour_frame)
  cv2.imshow(winName5, raw_contour_frame)

  key = cv2.waitKey(10)
  if key == 27:
    #cv2.destroyWindow(winName)
    break

print "Goodbye"


