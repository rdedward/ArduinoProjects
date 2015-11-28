locx = -100
locy = -100
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

print newx,", ",newy

