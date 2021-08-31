import serial
from matplotlib import pyplot as plt
from matplotlib import animation
import numpy as np
import threading
import random
import time
import pandas as pd

ser = serial.Serial('COM4', 115200)
ser.flush()

def receive_data(byte, sign = True):
    temp = []
    data = None

    for _ in range(byte):
        a = int(ser.read(1).hex(), 16) & 0xff
        temp.append(a)
    try:
        if byte == 1:
            data = temp[0]
        elif byte == 2:
            data = temp[0] << 8 | temp[1]
        elif byte == 3:
            data = temp[0] << 16 | temp[1] << 8 | temp[2]
        elif byte == 4:
            data = temp[0] << 24 | temp[1] << 16 | temp[2] << 8 | temp[3]
        elif byte == 8:
            data = temp[0] << 56 | temp[1] << 48 | temp[2] << 40 | temp[3] << 32 | temp[4] << 24 | temp[5] << 16 |\
                   temp[6] << 8 | temp[7]

        if sign:
            data_sign = temp[0] >> 7
            if data_sign:
                data = (data & 0x7fffffff) - 2 ** 31
    except:
        print("Byte Error Occured using Receive_data")

    return data

i = 0
mode_list = []
takeoff_step_list = []
increase_throttle_list = []
takeoff_throttle_list = []
lidar_list = []
baro_list = []
altitude_setpoint_list = []
lat_setpoint_list = []
lon_setpoint_list = []
flight_mode_list = []
failsafe_flag_list = []

while True:
    i += 1
    if i % 50 == 0:
        now = time.localtime()
        timevar = time.strftime('%d%H%M%S', now)
        df = pd.DataFrame()
        df['mode'] = mode_list
        df['takeoff step'] = takeoff_step_list
        df['increse throttle'] = increase_throttle_list
        df['takeoff throttle'] = takeoff_throttle_list
        df['lidar'] = lidar_list
        df['baro'] = baro_list
        df['altitude setpoint'] = altitude_setpoint_list
        df['lat setpoint'] = lat_setpoint_list
        df['lon setpoint'] = lon_setpoint_list
        df['flight mode'] = flight_mode_list
        df['failsafe flag'] = failsafe_flag_list

        df.to_csv(f"data/{timevar}_mission_data.csv")


    a = int(ser.read(1).hex(), 16) #int(ser.read(1).hex(), 16)
    if a == 0x77:
        b = int(ser.read(1).hex(), 16)
        if b == 0x17:
            
            mode = receive_data(1, sign = False)
            flight_mode = receive_data(1, sign=False)
            failsafe_flag = receive_data(1, sign=False)
            takeoff_step = receive_data(1, sign=False)
            increase_throttle = receive_data(4, sign=False)
            takeoff_throttle = receive_data(4, sign= False)
            lat_setpoint = receive_data(8) / (10 ** 7)
            lon_setpoint = receive_data(8) / (10 ** 7)
            lidar = receive_data(4) / 100
            baro = receive_data(4) / 100
            altitude_setpoint = receive_data(4) / 100

            mode_list.append(mode)
            takeoff_step_list.append(takeoff_step)
            increase_throttle_list.append(increase_throttle)
            takeoff_throttle_list.append(takeoff_throttle)
            lidar_list.append(lidar)
            baro_list.append(baro)
            altitude_setpoint_list.append(altitude_setpoint)
            lat_setpoint_list.append(lat_setpoint)
            lon_setpoint_list.append(lon_setpoint)
            flight_mode_list.append(flight_mode)
            failsafe_flag_list.append(failsafe_flag)

            print(f'mode : {mode}\t takeoff_step : {takeoff_step}\t increase_throttle : {increase_throttle}\t '
                  f'takeoff_throttle : {takeoff_throttle}\t lidar : {lidar}\t baro : {baro} alt_set : {altitude_setpoint}\t'
                  f'lat_set : {lat_setpoint}\t lon_set : {lon_setpoint}\t nx_flight_mode : {flight_mode}\t failsafe : {failsafe_flag}')

            ser.reset_input_buffer()