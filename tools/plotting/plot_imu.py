#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import sys

if len(sys.argv) < 2:
  print("need csv file to plot")
  exit(1)
else:
  f = open(sys.argv[1], 'r')
  lines = f.readlines()
  f.close()

timu = []
imu = []

tacc = []
acc = []

for line in lines:
  parts = line.split(',')
  
  if parts[1].strip() == '5': # acc data
    tacc.append(int(parts[0].strip()))
    v = []
    for i in range(2,len(parts)):
      v.append(float(parts[i].strip()))
    acc.append(v)
    
  if parts[1].strip() == '6': # imu data
    timu.append(int(parts[0].strip()))
    v = []
    for i in range(2,len(parts)):
      v.append(float(parts[i].strip()))
    imu.append(v)


timu = np.array(timu)
imu =  np.array(imu)

tacc = np.array(tacc)
acc =  np.array(acc)

acc_tdif = np.max(np.diff(tacc))
imu_tdif = np.max(np.diff(timu))

print("acc max time diff: {} seconds".format(acc_tdif/1000))
print("imu max time diff: {} seconds".format(imu_tdif/1000))

plt.figure()                                                                           
plt.plot(tacc/1000, acc, marker='o')
plt.title("high g accel data (g)")
plt.xlabel("Time (seconds)")
plt.ylabel("Acceleration (m/s/s)")
plt.legend(['x','y','z'])
plt.show(block=False)

plt.figure()
plt.plot(timu/1000, imu[:,:3]/1000, marker='o')
plt.title("low g accel data (g)")
plt.xlabel("Time (seconds)")
plt.ylabel("Acceleration (m/s/s)")
plt.legend(['x','y','z'])
plt.show(block=False)

plt.figure()
plt.plot(timu/1000, imu[:,-3:], marker='o')
plt.title("gyro data (deg/s)")
plt.xlabel("Time (seconds)")
plt.ylabel("Angular velocity (deg/s)")
plt.legend(['x','y','z'])
plt.show(block=False)


#legend=['acc x', 'acc y', 'acc z', 'gyr x', 'gyr y', 'gyr z']


