#!/usr/bin/env python3

import io
import sys
import time
import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
from KBHit import KBHit
import argparse
import json

import pdb

parser = argparse.ArgumentParser(description='Unsupervised clustering of protein sequences for comparing similarity between species')
parser.add_argument('--port', 
                    type=str, 
                    default='',
                    help='serial connection to groundstation hardware')
                    
parser.add_argument('--plottc',
                    type=bool,
                    default=False,
                    help='If set to 1, plot TC data received')
                    
parser.add_argument('--plotprs',
                    type=bool,
                    default=False,
                    help='If set to 1, plot Tpressure data received')

                    
args = parser.parse_args()

if args.port == '':
  parser.print_help()
  sys.exit(1)

def clrscr():
    # Check if Operating System is Mac and Linux or Windows
   if os.name == 'posix':
      _ = os.system('clear')
   else:
      # Else Operating System is Windows (os.name = nt)
      _ = os.system('cls')
      

#if len(sys.argv) < 2:
#  print("Usage: {} /path/to/serial_port".format(sys.argv[0]))

ser = serial.Serial(args.port, 115200, timeout=5)  # open serial port

kb = KBHit()

if ser.is_open:
  print("Opened port {} at 115200".format(ser.name))
else:
  print("Error, could not open {}".format(args.port))
  sys.exit(1)
  

data = []

if args.plottc:
  fig_tc, ax_tc = plt.subplots()

if args.plotprs:
  fig_prs, ax_prs = plt.subplots()
  
  

def updatePlots(it):

  datalen = len(data)

  if args.plottc and datalen>1:
    labels = list(np.array(list(data[0]['tc'].items()))[:,0])
    numTc  = np.array(list(data[0]['tc'].items()))[:,0].shape[0]
    tcData = np.zeros((datalen,numTc))
    tax    = np.zeros((datalen))
    for i in range(0,datalen):
      #pdb.set_trace()
      tcData[i,:] = np.array(list(data[i]['tc'].items()))[:,1].astype(float)
      tax[i] = data[i]['time'] / 1000 
    ax_tc.clear()
    return ax_tc.plot(tax, tcData)
  
  if args.plotprs:
    pass
    
if args.plottc or args.plotprs
  ani = FuncAnimation(fig_tc, updatePlots, 1000)





while True:

  cc = ser.readline()
  try:
    if cc.decode('utf-8') != '':
      ccr = cc.decode('utf-8').replace('nan',"0")
      y = json.loads(ccr)
      data.append(y)
      print(y)
      updatePlots(data)
      plt.tight_layout()
      plt.show()
      
  except json.decoder.JSONDecodeError:
    if len(ccr) > 0:
      print("error, cant parse: {}".format(ccr))

  time.sleep(1)

  if kb.kbhit():
    c = kb.getch()
    if ord(c) == 27: # ESC
      break
    print(c)
    
    #line = sio.readline()
    #print(line)
#      if len(line)==0:
#        break

kb.set_normal_term()
  
  
#ser.close()

