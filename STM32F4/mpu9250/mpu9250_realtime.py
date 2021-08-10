# MPU9250 Simple Visualization Code
# In order for this to run, the mpu9250_i2c file needs to 
# be in the local folder

from mpu9250_i2c import *
import smbus,time,datetime
import numpy as np
import matplotlib.pyplot as plt
import threading

plt.style.use('ggplot') # matplotlib visual style setting

time.sleep(1) # wait for mpu9250 sensor to settle

# ii = 1000 # number of points
t1 = time.time() # for calculating sample rate
# prepping for visualization
mpu6050_str = ['accel-x','accel-y','accel-z','gyro-x','gyro-y','gyro-z']
AK8963_str = ['mag-x','mag-y','mag-z']
mpu6050_vec,AK8963_vec,t_vec = [],[],[]

print('recording data')

def mpu_read():
    while True:
        try:
            ax,ay,az,wx,wy,wz = mpu6050_conv() # read and convert mpu6050 data
            mx,my,mz = AK8963_conv() # read and convert AK8963 magnetometer data
        except:
            continue
        ii += 1
        t_vec.append(time.time()) # capture timestamp
        AK8963_vec.append([mx,my,mz])
        mpu6050_vec.append([ax,ay,az,wx,wy,wz])
        print('sample rate accel: {} Hz'.format((time.time()-t1)/ii)) # print the sample rate

