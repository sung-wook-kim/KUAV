import serial
from matplotlib import pyplot as plt
from matplotlib import animation
import numpy as np
import threading
import random
import time
import pandas as pd

ser = serial.Serial('COM7', 115200)
ser.flush()

i = 0
yaw_list = []
lat_list = []
lon_list = []
target_lat_list = []
target_lon_list = []
pitch_adjust_list = []
roll_adjust_list = []
while True:
    i+=1
    if i % 100 == 0:
        now = time.localtime()
        timevar = time.strftime('%d%H%M%S', now)
        df = pd.DataFrame()
        df['yaw'] = yaw_list
        df['pvt_lat'] = lat_list
        df['pvt_lon'] = lon_list
        df['target_lat'] = target_lat_list
        df['target_lon'] = target_lon_list
        df['pitch_adjust'] = pitch_adjust_list
        df['roll_adjust'] = roll_adjust_list
        df.to_csv(f"data/{timevar}_gps_data.csv")


    a = int(ser.read(1).hex(), 16) #int(ser.read(1).hex(), 16)
    if a == 0x77:
        b = int(ser.read(1).hex(), 16)
        if b == 0x17:
            
            volt_1 = int(ser.read(1).hex(), 16) & 0xff
            volt_2 = int(ser.read(1).hex(), 16) & 0xff
            volt_3 = int(ser.read(1).hex(), 16) & 0xff
            volt_4 = int(ser.read(1).hex(), 16) & 0xff

            num_sv = int(ser.read(1).hex(), 16) & 0xff

            bno080_yaw_1 = int(ser.read(1).hex(), 16) & 0xff
            bno080_yaw_2 = int(ser.read(1).hex(), 16) & 0xff
            bno080_yaw_3 = int(ser.read(1).hex(), 16) & 0xff
            bno080_yaw_4 = int(ser.read(1).hex(), 16) & 0xff

            pvt_lat_1 = int(ser.read(1).hex(), 16) & 0xff
            pvt_lat_2 = int(ser.read(1).hex(), 16) & 0xff
            pvt_lat_3 = int(ser.read(1).hex(), 16) & 0xff
            pvt_lat_4 = int(ser.read(1).hex(), 16) & 0xff
            
            pvt_lon_1 = int(ser.read(1).hex(), 16) & 0xff
            pvt_lon_2 = int(ser.read(1).hex(), 16) & 0xff
            pvt_lon_3 = int(ser.read(1).hex(), 16) & 0xff
            pvt_lon_4 = int(ser.read(1).hex(), 16) & 0xff
            
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
            yaw_sign = bno080_yaw_1 >> 7
            
            voltage = volt_1 << 24 | volt_2 << 16 | volt_3 << 8 | volt_4

            bno080_yaw = bno080_yaw_1 << 24 | bno080_yaw_2 << 16 | bno080_yaw_3 << 8 | bno080_yaw_4

            pvt_lat = pvt_lat_1 << 24 | pvt_lat_2 << 16 | pvt_lat_3 << 8 | pvt_lat_4
            pvt_lon = pvt_lon_1 << 24 | pvt_lon_2 << 16 | pvt_lon_3 << 8 | pvt_lon_4

            target_lat = target_lat_1 << 24 | target_lat_2 << 16 | target_lat_3 << 8 | target_lat_4
            target_lon = target_lon_1 << 24 | target_lon_2 << 16 | target_lon_3 << 8 | target_lon_4

            pitch_adjust = pitch_adjust_1 << 24 | pitch_adjust_2 << 16 | pitch_adjust_3 << 8 | pitch_adjust_4
            roll_adjust = roll_adjust_1 << 24 | roll_adjust_2 << 16 | roll_adjust_3 << 8 | roll_adjust_4

            if yaw_sign == 1: bno080_yaw = (bno080_yaw & 0x7fffffff) - 2 ** 31
            if pitch_sign == 1: pitch_adjust = (pitch_adjust & 0x7fffffff) - 2 ** 31
            if roll_sign == 1: roll_adjust = (roll_adjust & 0x7fffffff) - 2 ** 31

            print(f' {i} // num_sv : {num_sv}, bno080_yaw : {bno080_yaw} , lat_error : {target_lat - pvt_lat} , lon_error : {target_lon - pvt_lon} \
            , target_lat : {target_lat} , target_lon : {target_lon} , pitch_adjust : {pitch_adjust} , roll_adjust : {roll_adjust}, voltage : {voltage/100}')
            yaw_list.append(bno080_yaw)
            lat_list.append(pvt_lat)
            lon_list.append(pvt_lon)
            target_lat_list.append(target_lat)
            target_lon_list.append(target_lon)
            pitch_adjust_list.append(pitch_adjust)
            roll_adjust_list.append(roll_adjust)
            ser.reset_input_buffer()
