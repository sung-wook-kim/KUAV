import serial
from time import time
import time
import pandas as pd
import matplotlib.pyplot as plt
import threading
import struct
ser = serial.Serial('COM7', 115200)
ser.flush()

global lat_out_reference , lat_out_meas_value , lon_out_reference , lon_out_meas_value,i , my_checksum

lat_out_reference = 0 
lat_out_meas_value = 0
lon_out_reference = 0 
lon_out_meas_value = 0
my_checksum = 0xffffffff
i = 0

lat_out_reference_list = []
lat_out_meas_value_list = []
lat_out_error_list = []
lat_out_error_deriv_list = []
lat_out_error_sum_list = []
lat_out_p_result_list = []
lat_out_i_result_list = []
lat_out_d_result_list = []
lat_out_pid_result_list = []

lat_in_reference_list = []
lat_in_meas_value_list = []
lat_in_error_list = []
lat_in_error_deriv_list = []
lat_in_error_sum_list = []
lat_in_p_result_list = []
lat_in_i_result_list = []
lat_in_d_result_list = []
lat_in_pid_result_list = []


lon_out_reference_list = []
lon_out_meas_value_list = []
lon_out_error_list = []
lon_out_error_deriv_list = []
lon_out_error_sum_list = []
lon_out_p_result_list = []
lon_out_i_result_list = []
lon_out_d_result_list = []
lon_out_pid_result_list = []

lon_in_reference_list = []
lon_in_meas_value_list = []
lon_in_error_list = []
lon_in_error_deriv_list = []
lon_in_error_sum_list = []
lon_in_p_result_list = []
lon_in_i_result_list = []
lon_in_d_result_list = []
lon_in_pid_result_list = []

lat_gps_list = []
pvt_lat_list = []

lon_gps_list = []
pvt_lon_list = []

roll_adjust_list = []
pitch_adjust_list = []

motor_one_list = []
motor_two_list = []
motor_three_list = []
motor_four_list = []
yaw_list = []


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
    global lat_out_reference , lat_out_meas_value , lon_out_meas_value , lon_out_reference ,i , my_checksum
    while True:
        i+=1
        if i % 100 == 0:
            now = time.localtime()
            timevar = time.strftime('%d%H%M', now)
            df = pd.DataFrame()
            df['lat_out_reference'] = lat_out_reference_list
            df['lat_out_meas_value'] = lat_out_meas_value_list
            df['lat_out_error'] = lat_out_error_list
            df['lat_out_error_deriv'] = lat_out_error_deriv_list
            df['lat_out_error_sum'] = lat_out_error_sum_list
            df['lat_out_p_result'] = lat_out_p_result_list
            df['lat_out_i_reult'] = lat_out_i_result_list
            df['lat_out_d_result'] = lat_out_d_result_list
            df['lat_out_pid_result'] = lat_out_pid_result_list

            df['lat_in_reference'] = lat_in_reference_list
            df['lat_in_meas_value'] = lat_in_meas_value_list
            df['lat_in_error'] = lat_in_error_list
            df['lat_in_error_deriv'] = lat_in_error_deriv_list
            df['lat_in_error_sum'] = lat_in_error_sum_list
            df['lat_in_p_result'] = lat_in_p_result_list
            df['lat_in_i_reult'] = lat_in_i_result_list
            df['lat_in_d_result'] = lat_in_d_result_list
            df['lat_in_pid_result'] = lat_in_pid_result_list

            df['lat_gps'] = lat_gps_list
            df['pvt_lat'] = pvt_lat_list

            df['lon_out_reference'] = lon_out_reference_list
            df['lon_out_meas_value'] = lon_out_meas_value_list
            df['lon_out_error'] = lon_out_error_list
            df['lon_out_error_deriv'] = lon_out_error_deriv_list
            df['lon_out_error_sum'] = lon_out_error_sum_list
            df['lon_out_p_result'] = lon_out_p_result_list
            df['lon_out_i_reult'] = lon_out_i_result_list
            df['lon_out_d_result'] = lon_out_d_result_list
            df['lon_out_pid_result'] = lon_out_pid_result_list

            df['lon_in_reference'] = lon_in_reference_list
            df['lon_in_meas_value'] = lon_in_meas_value_list
            df['lon_in_error'] = lon_in_error_list
            df['lon_in_error_deriv'] = lon_in_error_deriv_list
            df['lon_in_error_sum'] = lon_in_error_sum_list
            df['lon_in_p_result'] = lon_in_p_result_list
            df['lon_in_i_reult'] = lon_in_i_result_list
            df['lon_in_d_result'] = lon_in_d_result_list
            df['lon_in_pid_result'] = lon_in_pid_result_list

            df['lon_gps'] = lon_gps_list
            df['pvt_lon'] = pvt_lon_list

            df['roll_adjust'] = roll_adjust_list
            df['pitch_adjust'] = pitch_adjust_list

            df['motor_one'] = motor_one_list
            df['motor_two'] = motor_two_list
            df['motor_three'] = motor_three_list
            df['motor_four'] = motor_four_list

            df['yaw'] = yaw_list
            df.to_csv(f"pid_data_{timevar}.csv")
            print("Data is saved in data folder")
        
        my_checksum = 0xffffffff
        a = int(ser.read(1).hex(),16)
        if a == 0x11:
            b = int(ser.read(1).hex(),16)
            if b == 0x03:
                my_checksum -= 0x11
                my_checksum -= 0x03
                lat_out_reference = receive_data(4)
                lat_out_meas_value = receive_data(4)
                print(lat_out_meas_value)
                lat_out_error = receive_data(4)
                lat_out_error_deriv = receive_data(4)
                lat_out_error_sum = receive_data(4)
                lat_out_p_result = receive_data(4)
                lat_out_i_result = receive_data(4)
                lat_out_d_result = receive_data(4)
                lat_out_pid_result = receive_data(4)
                lat_in_reference = receive_data(4)
                lat_in_meas_value = receive_data(4)
                lat_in_error = receive_data(4)
                lat_in_error_deriv = receive_data(4)
                lat_in_error_sum = receive_data(4)
                lat_in_p_result = receive_data(4)
                lat_in_i_result = receive_data(4)
                lat_in_d_result = receive_data(4)
                lat_in_pid_result = receive_data(4)

                lat_gps = receive_data(8)
                pvt_lat = receive_data(8)

                lon_out_reference = receive_data(4)
                lon_out_meas_value = receive_data(4)
                lon_out_error = receive_data(4)
                lon_out_error_deriv = receive_data(4)
                lon_out_error_sum = receive_data(4)
                lon_out_p_result = receive_data(4)
                lon_out_i_result = receive_data(4)
                lon_out_d_result = receive_data(4)
                lon_out_pid_result = receive_data(4)

                lon_in_reference = receive_data(4)
                lon_in_meas_value = receive_data(4)
                lon_in_error = receive_data(4)
                lon_in_error_deriv = receive_data(4)
                lon_in_error_sum = receive_data(4)
                lon_in_p_result = receive_data(4)
                lon_in_i_result = receive_data(4)
                lon_in_d_result = receive_data(4)
                lon_in_pid_result = receive_data(4)

                lon_gps = receive_data(8)
                pvt_lon = receive_data(8)


                roll_adjust = receive_data(4)
                pitch_adjust = receive_data(4)

                motor_one = receive_data(4 , sign=False)
                motor_two = receive_data(4, sign=False)
                motor_three = receive_data(4, sign=False)
                motor_four = receive_data(4, sign=False)

                yaw = receive_data(4)                

                alt = receive_data(4)
                target_alt = receive_data(4)
                error = receive_data(4)
                throttle = receive_data(2)
                pid_result = receive_data(4)
                voltage = receive_data(4)

                checksum_1 = int(ser.read(1).hex(), 16) & 0xff
                checksum_2 = int(ser.read(1).hex(), 16) & 0xff
                checksum_3 = int(ser.read(1).hex(), 16) & 0xff
                checksum_4 = int(ser.read(1).hex(), 16) & 0xff
                
                checksum = checksum_1 << 24 | checksum_2 << 16 | checksum_3 << 8 | checksum_4 

                if checksum == my_checksum: 
                    lat_out_reference_list.append(lat_out_reference)
                    lat_out_meas_value_list.append(lat_out_meas_value)
                    lat_out_error_list.append(lat_out_error)
                    lat_out_error_deriv_list.append(lat_out_error_deriv)
                    lat_out_error_sum_list.append(lat_out_error_sum)
                    lat_out_p_result_list.append(lat_out_p_result)
                    lat_out_i_result_list.append(lat_out_i_result)
                    lat_out_d_result_list.append(lat_out_d_result)
                    lat_out_pid_result_list.append(lat_out_pid_result)

                    lat_in_reference_list.append(lat_in_reference)
                    lat_in_meas_value_list.append(lat_in_meas_value)
                    lat_in_error_list.append(lat_in_error)
                    lat_in_error_deriv_list.append(lat_in_error_deriv)
                    lat_in_error_sum_list.append(lat_in_error_sum)
                    lat_in_p_result_list.append(lat_in_p_result)
                    lat_in_i_result_list.append(lat_in_i_result)
                    lat_in_d_result_list.append(lat_in_d_result)
                    lat_in_pid_result_list.append(lat_in_pid_result)

                    lat_gps_list.append(lat_gps)
                    pvt_lat_list.append(pvt_lat)
                    

                    lon_out_reference_list.append(lon_out_reference)
                    lon_out_meas_value_list.append(lon_out_meas_value)
                    lon_out_error_list.append(lon_out_error)
                    lon_out_error_deriv_list.append(lon_out_error_deriv)
                    lon_out_error_sum_list.append(lon_out_error_sum)
                    lon_out_p_result_list.append(lon_out_p_result)
                    lon_out_i_result_list.append(lon_out_i_result)
                    lon_out_d_result_list.append(lon_out_d_result)
                    lon_out_pid_result_list.append(lon_out_pid_result)

                    lon_in_reference_list.append(lon_in_reference)
                    lon_in_meas_value_list.append(lon_in_meas_value)
                    lon_in_error_list.append(lon_in_error)
                    lon_in_error_deriv_list.append(lon_in_error_deriv)
                    lon_in_error_sum_list.append(lon_in_error_sum)
                    lon_in_p_result_list.append(lon_in_p_result)
                    lon_in_i_result_list.append(lon_in_i_result)
                    lon_in_d_result_list.append(lon_in_d_result)
                    lon_in_pid_result_list.append(lon_in_pid_result)

                    lon_gps_list.append(lon_gps)
                    pvt_lon_list.append(pvt_lon)

                    roll_adjust_list.append(roll_adjust)
                    pitch_adjust_list.append(pitch_adjust)

                    motor_one_list.append(motor_one)
                    motor_two_list.append(motor_two)
                    motor_three_list.append(motor_three)
                    motor_four_list.append(motor_four)
                    yaw_list.append(yaw)
                    # print(f'reference\tmeas_value\terror\terror_deriv\terror_sum\tp_result\ti_result\td_result\tpid_result\n')
                    # print(f'-------------------------------------------------------------------------------------------------\n')
                    # print(f'{pvt_lat} {lat_gps}\t{out_reference}\t{out_meas_value}\t{out_error}\t{out_error_deriv}\t{out_error_sum}\t{out_p_result}\t{out_i_result}\t{out_d_result}\t{out_pid_result}')
                    # print(f'{in_reference}\t{in_meas_value}\t{in_error}\t{in_error_deriv}\t{in_error_sum}\t{in_p_result}\t{in_i_result}\t{in_d_result}\t{in_pid_result}')
                    # print(f'{in_reference}\t{in_meas_value}\t{in_error}\t{in_error_deriv}\t{in_error_sum}\t{in_p_result}\
                    print(f'throttle :  {throttle} , voltage =  {voltage / 100}')
                    ser.reset_input_buffer()

thread1 = threading.Thread(target = connect)
thread1.start()

j = 0
while True:
    if lat_out_meas_value != 0 and lon_out_meas_value !=0:
        plt.scatter(lon_out_meas_value,lat_out_meas_value)
        if (lat_out_reference != lat_out_meas_value) or(lon_out_reference != lon_out_meas_value):
            plt.scatter(lon_out_reference , lat_out_reference , c= 'k' , s = 50)
        plt.xlabel('lat')
        plt.ylabel('lon')
        plt.pause(0.05)
        j+=1
        #print("one scatter", lat , lon , target_lat , target_lon)
plt.show()