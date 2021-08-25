import serial
from time import time
import time
import pandas as pd
import matplotlib.pyplot as plt
import threading
import struct
ser = serial.Serial('COM7', 115200)
ser.flush()

global yaw, numSV, fixtype, lat_gps , lon_gps , lat_waypoint , lon_waypoint , my_checksum , lat_gps_li , lon_gps_li , lat_waypoint_li , lon_waypoint_li

yaw = 0
numSV = 0
fixtype = 0
lat_gps = 0 
lat_waypoint = 0
lon_gps = 0 
lon_waypoint = 0

my_checksum = 0xffffffff
i = 0

yaw_li = []
numSV_li = []
fixtype_li = [] 
lat_gps_li = []
lon_gps_li = []
lat_waypoint_li = []
lon_waypoint_li = []
pitch_li = []
roll_li = []

def receive_data(byte, sign = True):
    global my_checksum
    temp = []
    data = None

    for _ in range(byte):
        a = int(ser.read(1).hex(), 16) & 0xff
        temp.append(a)
        my_checksum -= a
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
            data = temp[0] << 56 | temp[1] << 48 | temp[2] << 40 | temp[3] << 32 | temp[4] << 24 | temp[5] << 16 | temp[6] << 8 | temp[7]

        if sign :
            data_sign = temp[0] >> 7
            if data_sign:
                data = (data & 0x7fffffff) - 2 ** 31

    except:
        print("Byte Error Occured using Receive_data")

    return data

def connect():
    global my_checksum, yaw, numSV, fixtype lat_gps , lon_gps , lat_waypoint , lon_waypoint , i , lat_gps_li , lon_gps_li , lat_waypoint_li , lon_waypoint_li, numSV_li, fixtype_li
    while True:
        i+=1
        if i % 100 == 0:
            now = time.localtime()
            timevar = time.strftime('%d%H%M', now)
            df = pd.DataFrame()

            df['yaw'] = yaw_li
            df['numSV'] = numSV_li
            df['fixtype'] = fixtype_li
            df['lat'] = lat_gps_li
            df['lon'] = lon_gps_li
            df['lat_waypoint'] = lat_waypoint_li
            df['lon_waypoint'] = lon_waypoint_li
            df['pitch'] = pitch_li
            df['roll'] = roll_li
            df.to_csv(f"pid_data_{timevar}.csv")
            print("Data is saved in data folder")
        
        my_checksum = 0xffffffff
        a = int(ser.read(1).hex(),16)
        if a == 0x77:
            b = int(ser.read(1).hex(),16)
            if b == 0x17:
                my_checksum -= 0x77
                my_checksum -= 0x17
                volatge = receive_data(4)
                numSV = receive_data(1,sign=False)
                yaw = receive_data(4)
                lat_gps = receive_data(8)
                lon_gps = receive_data(8)
                lat_waypoint = receive_data(8)
                lon_waypoint = receive_data(8)
                pitch_adjust = receive_data(4)
                roll_adjust = receive_data(4)
                fixtype = receive_data(1,sign = False)
                
                checksum_1 = int(ser.read(1).hex(), 16) & 0xff
                checksum_2 = int(ser.read(1).hex(), 16) & 0xff
                checksum_3 = int(ser.read(1).hex(), 16) & 0xff
                checksum_4 = int(ser.read(1).hex(), 16) & 0xff
                
                checksum = checksum_1 << 24 | checksum_2 << 16 | checksum_3 << 8 | checksum_4
                print(f'lat = {lat_gps}, lon = {lon_gps}, target_lat = {lat_waypoint},target_lon = {lon_waypoint} ,yaw = {yaw}, vol = {volatge / 100} , numSV = {numSV}')
                ser.reset_input_buffer()
                if checksum == my_checksum:
                    yaw_li.append(yaw)
                    numSV_li.append(numSV_li)
                    lat_gps_li.append(lat_gps)
                    lon_gps_li.append(lon_gps)
                    lat_waypoint_li.append(lat_waypoint)
                    lon_waypoint_li.append(lon_waypoint)
                    pitch_li.append(pitch_adjust)
                    roll_li.append(roll_adjust)
                    fixtype_li.append(fixtype)

                    # print(f'reference\tmeas_value\terror\terror_deriv\terror_sum\tp_result\ti_result\td_result\tpid_result\n')
                    # print(f'-------------------------------------------------------------------------------------------------\n')
                    # print(f'{pvt_lat} {lat_gps}\t{out_reference}\t{out_meas_value}\t{out_error}\t{out_error_deriv}\t{out_error_sum}\t{out_p_result}\t{out_i_result}\t{out_d_result}\t{out_pid_result}')
                    # print(f'{in_reference}\t{in_meas_value}\t{in_error}\t{in_error_deriv}\t{in_error_sum}\t{in_p_result}\t{in_i_result}\t{in_d_result}\t{in_pid_result}')
                    # print(f'{in_reference}\t{in_meas_value}\t{in_error}\t{in_error_deriv}\t{in_error_sum}\t{in_p_result}\
                    # print("check")
                    ser.reset_input_buffer()

thread1 = threading.Thread(target = connect)
thread1.start()

j = 0
time.sleep(3)
while True:
    if lat_gps_li[-1] != 0 and lon_gps_li[-1] !=0:
        plt.scatter(lon_gps_li[-1],lat_gps_li[-1])
        if (lat_waypoint_li[-1] != lat_gps_li[-1]) or(lon_waypoint_li[-1] != lon_gps_li[-1]):
            plt.scatter(lon_waypoint_li[-1] , lat_waypoint_li[-1] , c= 'k' , s = 100)
        plt.xlabel('lat')
        plt.ylabel('lon')
        plt.pause(0.05)
        j+=1
        #print("one scatter", lat , lon , target_lat , target_lon)
plt.show()