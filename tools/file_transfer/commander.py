import io
import sys
import time
import serial

if len(sys.argv) < 3:
  print("Usage: {} serial_port command".format(sys.argv[0]))

ser = serial.Serial(sys.argv[1], 115200, timeout=3)  # open serial port
cmd = sys.argv[2]

if ser.is_open:
  
  sio = io.TextIOWrapper(io.BufferedRWPair(ser, ser))
  
  print("Opened port {} at 115200".format(ser.name))
  sio.write("{}\r\n".format(cmd))     # write a string
  sio.flush()
  
  while True:
    line = sio.readline()
    print(line,end='')
    if len(line)==0:
      break

ser.close()

