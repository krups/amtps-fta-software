#!/usr/bin/env python3

import io
import sys
import time
import serial
from KBHit import KBHit

def clrscr():
    # Check if Operating System is Mac and Linux or Windows
   if os.name == 'posix':
      _ = os.system('clear')
   else:
      # Else Operating System is Windows (os.name = nt)
      _ = os.system('cls')
      

#if len(sys.argv) < 2:
#  print("Usage: {} /path/to/serial_port".format(sys.argv[0]))

#ser = serial.Serial(sys.argv[1], 115200, timeout=0.5)  # open serial port

kb = KBHit()

#if ser.is_open:
  
  #sio = io.TextIOWrapper(io.BufferedRWPair(ser, ser))
  
  #print("Opened port {} at 115200".format(ser.name))
  #sio.write("{}\r\n".format(cmd))     # write a string
  #sio.flush()
  
while True:
  if kb.kbhit():
    c = kb.getch()
    if ord(c) == 27: # ESC
      break
    print(c)
    
    #line = sio.readline()
    #print(line,end='')
#      if len(line)==0:
#        break

kb.set_normal_term()
  
  
#ser.close()

