import sys
import serial
import time

test = 1
ser = serial.Serial('/dev/ttyACM0',9600)
print len(sys.argv)


if (len(sys.argv) == 5):
  print "Using speed on ", sys.argv[4]
  start = int(sys.argv[1])
  end = int(sys.argv[2])
  print "end is ", end
  delay = float(sys.argv[3])
  pos = start
  print type(pos)
  rot = 1
  while(test == 1):
    print "writing" + str(pos)+'\n'+str(pos)
    if (int(sys.argv[4]) == 1):
      ser.write(str(pos)+'\n'+"10" + '\n')
      print "driving motor 1"
    else:
      ser.write("10"+'\n'+str(pos) + '\n')
      print "driving motor 2"
    if (rot == 1):
      pos = pos+1
    else:
      pos = pos - 1
    if (pos > end and rot == 1):
      rot = -1
    if (pos < start and rot == -1):
      rot = 1
    time.sleep(delay)

elif (len(sys.argv) == 4):
  print "Using speed"
  start = int(sys.argv[1])
  end = int(sys.argv[2])
  print "end is ", end
  delay = float(sys.argv[3])
  pos = start
  print type(pos)
  rot = 1
  while(test == 1):
    print "writing" + str(pos)+'\n'+str(pos)
    ser.write(str(pos)+'\n'+str(pos) + '\n')
    if (rot == 1):
      pos = pos+1
    else:
      pos = pos - 1
    if (pos > end and rot == 1):
      rot = -1
    if (pos < start and rot == -1):
      rot = 1
    time.sleep(delay)

elif (len(sys.argv) == 3):
  print "Using Arguments"
  ser.write(sys.argv[1]+'\n'+sys.argv[2])
else:
  ser.write('20'+'\n'+'20')
