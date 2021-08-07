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
global alt , target , error ,value, d_result , voltage
alt = 0
target = 0
error = 0
value = 0
d_result = 0
voltage = 0

ser = serial.Serial('COM7', 115200)
ser.flush()

#
    # for save
i = 0
now = time.localtime()
timevar = time.strftime('%d%H%M%S', now)

while True:
    i+=1

    a = int(ser.read(1).hex(), 16)
    if a == 0x77:
        b = int(ser.read(1).hex(), 16)
        if b == 0x17:
            num_sv = int(ser.read(1).hex(), 16) & 0xff
            
            pvt_lat_1 = int(ser.read(1).hex(), 16) & 0xff
            pvt_lat_2 = int(ser.read(1).hex(), 16) & 0xff
            pvt_lat_3 = int(ser.read(1).hex(), 16) & 0xff
            pvt_lat_4 = int(ser.read(1).hex(), 16) & 0xff
            
            pvt_lon_1 = int(ser.read(1).hex(), 16) & 0xff
            pvt_lon_2 = int(ser.read(1).hex(), 16) & 0xff
            pvt_lon_3 = int(ser.read(1).hex(), 16) & 0xff
            pvt_lon_4 = int(ser.read(1).hex(), 16) & 0xff
            
            l_lat_1 = int(ser.read(1).hex(), 16) & 0xff
            l_lat_2 = int(ser.read(1).hex(), 16) & 0xff
            l_lat_3 = int(ser.read(1).hex(), 16) & 0xff
            l_lat_4 = int(ser.read(1).hex(), 16) & 0xff

            l_lon_1 = int(ser.read(1).hex(), 16) & 0xff
            l_lon_2 = int(ser.read(1).hex(), 16) & 0xff
            l_lon_3 = int(ser.read(1).hex(), 16) & 0xff
            l_lon_4 = int(ser.read(1).hex(), 16) & 0xff

            bno080_yaw_1 = int(ser.read(1).hex(), 16) & 0xff
            bno080_yaw_2 = int(ser.read(1).hex(), 16) & 0xff
            bno080_yaw_3 = int(ser.read(1).hex(), 16) & 0xff
            bno080_yaw_4 = int(ser.read(1).hex(), 16) & 0xff

            yaw_sign = bno080_yaw_1 >> 7
            
            pvt_lat = pvt_lat_1 << 24 | pvt_lat_2 << 16 | pvt_lat_3 << 8 | pvt_lat_4
            pvt_lon = pvt_lon_1 << 24 | pvt_lon_2 << 16 | pvt_lon_3 << 8 | pvt_lon_4
            l_lat = l_lat_1 << 24 | l_lat_2 << 16 | l_lat_3 << 8 | l_lat_4
            l_lon = l_lon_1 << 24 | l_lon_2 << 16 | l_lon_3 << 8 | l_lon_4
            bno080_yaw = bno080_yaw_1 << 24 | bno080_yaw_2 << 16 | bno080_yaw_3 << 8 | bno080_yaw

            if yaw_sign == 1: bno080_yaw = (bno080_yaw & 0x7fffffff) - 2 ** 31
    
            print(num_sv, pvt_lat, pvt_lon , l_lat, l_lon)


