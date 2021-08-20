import serial
from matplotlib import pyplot as plt
from matplotlib import animation
import numpy as np
import threading
import random
import time
import pandas as pd
from collections import deque
import math

# 네가지 값 프린트 , 전체변경으로 value 값을 원하는 변수로 변경 하세요
# alt , target , error global declaration + value
global lat , lon , target_lat , target_lon , pitch , roll , yaw
lat = 0
lon = 0
target_lat = 0
target_lon = 0
pitch = 0
roll = 0
yaw = 0

ser = serial.Serial('COM7', 115200)
ser.flush()

def get_metres_location(original_location, other_location):
    """Convert distance from degrees longitude-latitude to meters.
        Takes the two points described by (Lat,Lon) in degrees
        and converts it into distances *dx(north distance)* and *dy(east distance)* in meters.
        returns (float, float)
    """
    earth_radius = 6378137.0  # Radius of "spherical" earth
    # Coordinate offsets in radians
    dLatLon = [other_location[0] - original_location[0], other_location[1] - original_location[1]]
    dx = dLatLon[0] * earth_radius * math.pi / 180
    dy = dLatLon[1] * (earth_radius * math.cos(math.pi * original_location[0] / 180)) * math.pi / 180

    return [dx, dy]

# you have to receive 18bytes
def connect():
    global lat , lon , target_lat , target_lon , pitch , roll , yaw
    # for save
    i = 0
    now = time.localtime()
    timevar = time.strftime('%d%H%M%S', now)
    # list
    alt_li = []
    target_li = []
    error_li = []
    value_li = []
    pitch_li = []
    roll_li = []
    while True:
        i+=1
        # 100개의 데이터 마다 저장
        if i%100 == 0:
            df = pd.DataFrame()
            df['lat'] = alt_li
            df['lon'] = target_li
            df['target_lat'] = error_li
            df['target_lon'] = value_li
            df['pitch_adjust'] = pitch_li
            df['roll_adjust'] = roll_li
            df.to_csv(f"Data/{timevar}_alt_data.csv")

        a = int(ser.read(1).hex(), 16)
        if a == 0x77:
            b = int(ser.read(1).hex(), 16)
            if b == 0x17:
                volt_1 = int(ser.read(1).hex(), 16) & 0xff
                volt_2 = int(ser.read(1).hex(), 16) & 0xff
                volt_3 = int(ser.read(1).hex(), 16) & 0xff
                volt_4 = int(ser.read(1).hex(), 16) & 0xff

                count_1 = int(ser.read(1).hex(), 16) & 0xff
                
                yaw_1 = int(ser.read(1).hex(), 16) & 0xff
                yaw_2 = int(ser.read(1).hex(), 16) & 0xff
                yaw_3 = int(ser.read(1).hex(), 16) & 0xff
                yaw_4 = int(ser.read(1).hex(), 16) & 0xff
                
                lat_1 = int(ser.read(1).hex(), 16) & 0xff
                lat_2 = int(ser.read(1).hex(), 16) & 0xff
                lat_3 = int(ser.read(1).hex(), 16) & 0xff
                lat_4 = int(ser.read(1).hex(), 16) & 0xff

                lon_1 = int(ser.read(1).hex(), 16) & 0xff
                lon_2 = int(ser.read(1).hex(), 16) & 0xff
                lon_3 = int(ser.read(1).hex(), 16) & 0xff
                lon_4 = int(ser.read(1).hex(), 16) & 0xff

                target_lat_1 = int(ser.read(1).hex(), 16) & 0xff
                target_lat_2 = int(ser.read(1).hex(), 16) & 0xff
                target_lat_3 = int(ser.read(1).hex(), 16) & 0xff
                target_lat_4 = int(ser.read(1).hex(), 16) & 0xff

                target_lon_1 = int(ser.read(1).hex(), 16) & 0xff
                target_lon_2 = int(ser.read(1).hex(), 16) & 0xff
                target_lon_3 = int(ser.read(1).hex(), 16) & 0xff
                target_lon_4 = int(ser.read(1).hex(), 16) & 0xff

                pitch_adjust_1 = int(ser.read(1).hex(), 16) & 0xff
                pitch_adjust_2 = int(ser.read(1).hex(), 16) & 0xff
                pitch_adjust_3 = int(ser.read(1).hex(), 16) & 0xff
                pitch_adjust_4 = int(ser.read(1).hex(), 16) & 0xff

                roll_adjust_1 = int(ser.read(1).hex(), 16) & 0xff
                roll_adjust_2 = int(ser.read(1).hex(), 16) & 0xff
                roll_adjust_3 = int(ser.read(1).hex(), 16) & 0xff
                roll_adjust_4 = int(ser.read(1).hex(), 16) & 0xff

                pitch_sign = pitch_adjust_1 >> 7
                roll_sign = roll_adjust_1 >> 7
                yaw_sign = yaw_1 >> 7                
                voltage = volt_1 << 24 | volt_2 << 16 | volt_3 << 8 | volt_4
                yaw = yaw_1 << 24 | yaw_2 << 16 | yaw_3 << 8 | yaw_4
                lat = lat_1 << 24 | lat_2 << 16 | lat_3 << 8 | lat_4
                lon = lon_1 << 24 | lon_2 << 16 | lon_3 << 8 | lon_4
                target_lat = target_lat_1 << 24 | target_lat_2 << 16 | target_lat_3 << 8 | target_lat_4
                target_lon = target_lon_1 << 24 | target_lon_2 << 16 | target_lon_3 << 8 | target_lon_4
                pitch = pitch_adjust_1 << 24 | pitch_adjust_2 << 16 | pitch_adjust_3 << 8 | pitch_adjust_4
                                

                if pitch_sign == 1: pitch = (pitch & 0x7fffffff) - 2 ** 31
                if roll_sign == 1 : roll = (roll & 0x7fffffff) - 2 ** 31
                if yaw_sign == 1: yaw = (yaw & 0x7fffffff) - 2 ** 31
                print(f'lat : {lat} , lon : {lon} , yaw : {yaw} , voltage : {voltage / 100}')
                alt_li.append(lat)
                target_li.append(lon)
                error_li.append(target_lat)
                value_li.append(target_lon)
                pitch_li.append(pitch)
                roll_li.append(roll)
fig = plt.figure(1)
# h , w 간격 조절
fig.subplots_adjust(hspace=0.4, wspace=0.2)

ax = plt.subplot(231, xlim=(0, 50), ylim=(37.51, 37.52))
ax.set_title("lat")
ax_2 = plt.subplot(232, xlim=(0, 50), ylim=(37.517, 37.52))
ax_2.set_title("target_lat")
ax_3 = plt.subplot(233, xlim=(37.51, 37.52), ylim=(126., 126.8000))
ax_3.set_title("lat_lon")
ax_4 = plt.subplot(234, xlim=(0, 50), ylim=(126.87, 5000))
ax_4.set_title("target_lon")
ax_5 = plt.subplot(235, xlim=(0, 50), ylim=(-2000, 5000))
ax_5.set_title("pitch_adjust")
ax_6 = plt.subplot(236, xlim=(0, 50), ylim=(-2000, 5000))
ax_6.set_title("rollP_adjust")

# 화면에 보여질 점의 개수 ( 길이 )
max_points = 50

line, = ax.plot(np.arange(max_points), 
                np.ones(max_points, dtype=np.float)*np.nan, lw=2)

line_2, = ax_2.plot(np.arange(max_points), 
                np.ones(max_points, dtype=np.float)*np. 7, lw=1,ms=1)

line_3, = ax_2.plot(np.arange(max_points),
                np.ones(max_points, dtype=np.float)*np.nan, lw=1,ms=1)

line_4, = ax_4.plot(np.arange(max_points), 
                np.ones(max_points, dtype=np.float)*np.nan, lw=1,ms=1)

line_5, = ax_5.plot(np.arange(max_points), 
                np.ones(max_points, dtype=np.float)*np.nan, lw=1,ms=1)

line_6, = ax_6.plot(np.arange(max_points), 
                np.ones(max_points, dtype=np.float)*np.nan, lw=1,ms=1)
def init():
    return line,

def init_2():
    return line_2

def init_3():
    return line_3

def init_4():
    return line_4

def init_5():
    return line_5

def init_6():
    return line_6

def animate(i):
    global lat 
    old_y = line.get_ydata()
    new_y = np.r_[old_y[1:], lat / 10**7]
    line.set_ydata(new_y)
    return line,

def animate_2(i):
    global target_lat
    old_y_2 = line_2.get_ydata()
    new_y_2 = np.r_[old_y_2[1:], target_lat / 10**7]
    line_2.set_ydata(new_y_2)

    return line_2

def animate_3(i):
    global lon
    old_y_3 = line_3.get_ydata()
    new_y_3 = np.r_[old_y_3[1:], lon]
    line_3.set_ydata(new_y_3)
    return line_3

# value를 다른 값으로 변경하여 사용
def animate_4(i):
    global target_lon
    old_y_4 = line_4.get_ydata()
    new_y_4 = np.r_[old_y_4[1:], target_lon]
    line_4.set_ydata(new_y_4)
    return line_4

def animate_5(i):
    global pitch
    old_y_5 = line_5.get_ydata()
    new_y_5 = np.r_[old_y_5[1:], pitch]
    line_5.set_ydata(new_y_5)
    return line_5

def animate_6(i):
    global roll
    old_y_6 = line_6.get_ydata()
    new_y_6 = np.r_[old_y_6[1:], roll]
    line_6.set_ydata(new_y_6)
    return line_6

thread1 = threading.Thread(target = connect)
thread1.start()

def gps_thread(figure):
    global lat, lon
    original = [lon,lat]
    dy_list = deque(maxlen = 50)
    dx_list = deque(maxlen = 50)

    while True:
        dx,dy = get_metres_location(original, (lat,lon))
        dx_list.append(dx)
        dy_list.append(dy)
        plt.scatter(dy_list,dx_list)
        plt.xlabel('East')
        plt.ylabel('North')
        figure.canvas.draw()
        figure.canvas.flush_events()
        plt.pause(0.1)
        plt.clf()

# frame -> 반복문에 들어가는 i의 값 , 의미없음 / interval = 실행단위 100ms
anim = animation.FuncAnimation(fig, animate, init_func=init, frames=200, interval=100, blit=False)
anim_2 = animation.FuncAnimation(fig, animate_2  , init_func= init_2 ,frames=200, interval=100, blit=False)
anim_3 = animation.FuncAnimation(fig, animate_3  , init_func= init_3 ,frames=200, interval=100, blit=False)
anim_4 = animation.FuncAnimation(fig, animate_4  , init_func= init_4 ,frames=200, interval=100, blit=False)
anim_5 = animation.FuncAnimation(fig, animate_5  , init_func= init_5 ,frames=200, interval=100, blit=False)
anim_6 = animation.FuncAnimation(fig, animate_6  , init_func= init_6 ,frames=200, interval=100, blit=False)
plt.show()
fig2 = plt.figure(1)
GPSthread = threading.Thread(target = gps_thread, daemon= True)
