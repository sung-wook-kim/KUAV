# MPU9250 Simple Visualization Code
# In order for this to run, the mpu9250_i2c file needs to 
# be in the local folder

from mpu9250_i2c import *
import smbus,time,datetime
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
import threading
from collections import deque

plt.style.use('ggplot') # matplotlib visual style setting

time.sleep(1) # wait for mpu9250 sensor to settle

# ii = 1000 # number of points
t1 = time.time() # for calculating sample rate
# prepping for visualization
mpu6050_str = ['accel-x','accel-y','accel-z','gyro-x','gyro-y','gyro-z']
AK8963_str = ['mag-x','mag-y','mag-z']
ax,ay,az,wx,wy,wz,mx,my,mz= deque(maxlen=50),deque(maxlen=50),deque(maxlen=50),deque(maxlen=50),deque(maxlen=50),deque(maxlen=50),deque(maxlen=50),deque(maxlen=50),deque(maxlen=50)

print('recording data')

def mpu_read():
    # global mpu6050_vec,AK8963_vec,t_vec
    global t1
    while True:
        try:
            ax_temp,ay_temp,az_temp,wx_temp,wy_temp,wz_temp = mpu6050_conv() # read and convert mpu6050 data
            ax.append(ax_temp)
            ay.append(ay_temp)
            az.append(az_temp)
            wx.append(wx_temp)
            wy.append(wy_temp)
            wz.append(wz_temp)
            mx_temp,my_temp,mz_temp = AK8963_conv() # read and convert AK8963 magnetometer data
            mx.append(mx_temp)
            my.append(my_temp)
            mz.append(mz_temp)
        except:
            continue
        t2 = time.time()
        print('sample rate accel: {} Hz'.format(1/(t2-t1))) # print the sample rate
        t1 = t2


fig = plt.figure()
fig.subplots_adjust(hspace=0.4, wspace=0.2)

ax_1 = plt.subplot(221, xlim=(0, 50), ylim=(-500, 500))
ax_1.set_title("accelation")
ax_2 = plt.subplot(222, xlim=(0, 50), ylim=(-500, 500))
ax_2.set_title("rotation")
ax_3 = plt.subplot(223, xlim=(0, 50), ylim=(-500, 500))
ax_3.set_title("direction")


max_points = 50

line1, = ax_1.plot(ax, lw=2, label='ax')
line2, = ax_1.plot(ay, lw=2, label='ay')
line3, = ax_1.plot(az, lw=2, label='az')

line11, = ax_2.plot(wx, lw=2, label='wx')
line12, = ax_2.plot(wy, lw=2, label='wy')
line13, = ax_2.plot(wz, lw=2, label='wz')

line21, = ax_3.plot(mx, lw=2, label='mx')
line22, = ax_3.plot(my, lw=2, label='my')
line23, = ax_3.plot(mz, lw=2, label='mz')                

def animate(i):
    line1.set_data(ax)
    line2.set_data(ay)
    line3.set_data(az)

def animate_2(i):
    line11.set_data(wx)
    line12.set_data(wy)
    line13.set_data(wz)

def animate_3(i):
    line21.set_data(mx)
    line22.set_data(my)
    line23.set_data(mz)

thread1 = threading.Thread(target = mpu_read)
thread1.start()

anim = animation.FuncAnimation(fig, animate, frames=200, interval=100, blit=False)
anim_2 = animation.FuncAnimation(fig, animate_2,frames=200, interval=100, blit=False)
anim_3 = animation.FuncAnimation(fig, animate_3,frames=200, interval=100, blit=False)
plt.show()