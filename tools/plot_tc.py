#!/usr/bin/env python3

# AMPTS TC packet plotter
# Matt Ruffner 2021
# University of Kentucky
#
# Input a TLM log file and it will plot stuffs

import os
import sys
import numpy as np
import matplotlib.pyplot as plt

from scipy import ndimage, misc

if len(sys.argv) < 2:
    print("Need input csv from CDH board")
    sys.exit(1)

numInputs = len(sys.argv)-1

data = []

f = open(sys.argv[1], 'r')
lines = f.readlines()
f.close()

temps = []
tempstax = []

prs = []
prstax = []

for line in lines:
  parts = line.split(',')
  
  if parts[1].strip() == '0': # tc data
    tempstax.append(int(parts[0].strip()))
    t = []
    for i in range(2,len(parts)):
      t.append(float(parts[i].strip()))
    temps.append(t)
    
  if parts[1].strip() == '1': # pressure data
    prstax.append(int(parts[0].strip()))
    p = []
    for i in range(2,len(parts)):
      p.append(float(parts[i].strip()))
    prs.append(p)


temps = np.array(temps)
tempstax  = np.array(tempstax)
prs = np.array(prs)
prstax = np.array(prstax)

plt.figure()
plt.plot(tempstax/1000, temps, linestyle='--', marker='o')
plt.legend(["TC {}".format(i) for i in range(1,13)])
plt.xlabel('Time (seconds)')
plt.ylabel('Temperature (deg. C)')
plt.title("Temperature")


plt.figure()
plt.plot(prstax/1000, prs, linestyle='--', marker='o')
plt.legend(["TC {}".format(i) for i in range(1,13)])
plt.xlabel('Time (seconds)')
plt.ylabel('Pressure (kPa)')
plt.title("Pressure")


plt.show()
