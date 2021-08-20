import serial
from matplotlib import pyplot as plt
from matplotlib import animation
import numpy as np
import threading
import random
import time
import pandas as pd

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
        # # 100개의 데이터 마다 저장
        # if i%100 == 0:
        #     df = pd.DataFrame()
        #     df['lat'] = alt_li
        #     df['lon'] = target_li
        #     df['target_lat'] = error_li
        #     df['target_lon'] = value_li
        #     df['pitch_adjust'] = pitch_li
        #     df['roll_adjust'] = roll_li
        #     df.to_csv(f"Data/{timevar}_alt_data.csv")

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
                volt = volt_1 << 24 | volt_2 << 16 | volt_3 << 8 | volt_4
                yaw = yaw_1 << 24 | yaw_2 << 16 | yaw_3 << 8 | yaw_4
                lat = lat_1 << 24 | lat_2 << 16 | lat_3 << 8 | lat_4
                lon = lon_1 << 24 | lon_2 << 16 | lon_3 << 8 | lon_4
                target_lat = target_lat_1 << 24 | target_lat_2 << 16 | target_lat_3 << 8 | target_lat_4
                target_lon = target_lon_1 << 24 | target_lon_2 << 16 | target_lon_3 << 8 | target_lon_4
                pitch = pitch_adjust_1 << 24 | pitch_adjust_2 << 16 | pitch_adjust_3 << 8 | pitch_adjust_4
                roll = roll_adjust_1 << 24 | roll_adjust_2 << 16 | roll_adjust_3 << 8 | roll_adjust_4
                                

                if pitch_sign == 1: pitch = (pitch & 0x7fffffff) - 2 ** 31
                if roll_sign == 1 : roll = (roll & 0x7fffffff) - 2 ** 31
                if yaw_sign == 1: yaw = (yaw & 0x7fffffff) - 2 ** 31
                print(f'pitch = {pitch} , roll = {roll} , yaw : {yaw} , sv : {count_1} ,  voltage : {volt / 100} ')
                alt_li.append(lat)
                target_li.append(lon)
                error_li.append(target_lat)
                value_li.append(target_lon)
                pitch_li.append(pitch)
                roll_li.append(roll)


thread1 = threading.Thread(target = connect)
thread1.start()

while True:
    if lat != 0 and lon !=0:
        plt.scatter(lat,lon)
        if lat != target_lat and lon != target_lon:
            plt.scatter(target_lat,target_lon,c  = 'k' , s = 50)
        plt.xlabel('lat')
        plt.ylabel('lon')
        plt.pause(0.01)
        #print("one scatter", lat , lon , target_lat , target_lon)
plt.show()


