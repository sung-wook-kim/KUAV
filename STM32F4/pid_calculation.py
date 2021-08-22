import serial
from time import time
import time
import pandas as pd

ser = serial.Serial('COM7', 115200)
ser.flush()

i = 0

out_reference_list = []
out_meas_value_list = []
out_error_list = []
out_error_deriv_list = []
out_error_sum_list = []
out_p_result_list = []
out_i_result_list = []
out_d_result_list = []
out_pid_result_list = []

in_reference_list = []
in_meas_value_list = []
in_error_list = []
in_error_deriv_list = []
in_error_sum_list = []
in_p_result_list = []
in_i_result_list = []
in_d_result_list = []
in_pid_result_list = []

lat_gps_list = []
pvt_lat_list = []

roll_adjust_list = []
pitch_adjust_list = []

motor_one_list = []
motor_two_list = []
motor_three_list = []
motor_four_list = []
def receive_data(byte, sign = True):
    temp = []
    data = None

    for _ in range(byte):
        temp.append(int(ser.read(1).hex(), 16) & 0xff)

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


while True:
    i+=1
    if i % 100 == 0:
        now = time.localtime()
        timevar = time.strftime('%d%H%M%S', now)
        df = pd.DataFrame()
        df['out_reference'] = out_reference_list
        df['out_meas_value'] = out_meas_value_list
        df['out_error'] = out_error_list
        df['out_error_deriv'] = out_error_deriv_list
        df['out_error_sum'] = out_error_sum_list
        df['out_p_result'] = out_p_result_list
        df['out_i_reult'] = out_i_result_list
        df['out_d_result'] = out_d_result_list
        df['out_pid_result'] = out_pid_result_list

        df['in_reference'] = in_reference_list
        df['in_meas_value'] = in_meas_value_list
        df['in_error'] = in_error_list
        df['in_error_deriv'] = in_error_deriv_list
        df['in_error_sum'] = in_error_sum_list
        df['in_p_result'] = in_p_result_list
        df['in_i_reult'] = in_i_result_list
        df['in_d_result'] = in_d_result_list
        df['in_pid_result'] = in_pid_result_list

        df['lat_gps'] = lat_gps_list
        df['pvt_lat'] = pvt_lat_list

        df['roll_adjust'] = roll_adjust_list
        df['pitch_adjust'] = pitch_adjust_list

        df['motor_one'] = motor_one_list
        df['motor_two'] = motor_two_list
        df['motor_three'] = motor_three_list
        df['motor_four'] = motor_four_list

        df.to_csv(f"data/pid_data_{timevar}.csv")
        print("Data is saved in data folder")

    a = int(ser.read(1).hex(),16)
    if a == 0x11:
        b = int(ser.read(1).hex(),16)
        if b == 0x03:
            out_reference = receive_data(4)
            out_meas_value = receive_data(4)
            out_error = receive_data(4)
            out_error_deriv = receive_data(4)
            out_error_sum = receive_data(4)
            out_p_result = receive_data(4)
            out_i_result = receive_data(4)
            out_d_result = receive_data(4)
            out_pid_result = receive_data(4)

            in_reference = receive_data(4)
            in_meas_value = receive_data(4)
            in_error = receive_data(4)
            in_error_deriv = receive_data(4)
            in_error_sum = receive_data(4)
            in_p_result = receive_data(4)
            in_i_result = receive_data(4)
            in_d_result = receive_data(4)
            in_pid_result = receive_data(4)

            lat_gps = receive_data(8)
            pvt_lat = receive_data(8)

            roll_adjust = receive_data(4)
            pitch_adjust = receive_data(4)

            motor_one = receive_data(4)
            motor_two = receive_data(4)
            motor_three = receive_data(4)
            motor_four = receive_data(4)
            
            ser.reset_input_buffer()

            out_reference_list.append(out_reference)
            out_meas_value_list.append(out_meas_value)
            out_error_list.append(out_error)
            out_error_deriv_list.append(out_error_deriv)
            out_error_sum_list.append(out_error_sum)
            out_p_result_list.append(out_p_result)
            out_i_result_list.append(out_i_result)
            out_d_result_list.append(out_d_result)
            out_pid_result_list.append(out_pid_result)

            in_reference_list.append(in_reference)
            in_meas_value_list.append(in_meas_value)
            in_error_list.append(in_error)
            in_error_deriv_list.append(in_error_deriv)
            in_error_sum_list.append(in_error_sum)
            in_p_result_list.append(in_p_result)
            in_i_result_list.append(in_i_result)
            in_d_result_list.append(in_d_result)
            in_pid_result_list.append(in_pid_result)

            lat_gps_list.append(lat_gps)
            pvt_lat_list.append(pvt_lat)

            roll_adjust_list.append(roll_adjust)
            pitch_adjust_list.append(pitch_adjust)

            motor_one_list.append(motor_one)
            motor_two_list.append(motor_two)
            motor_three_list.append(motor_three)
            motor_four_list.append(motor_four)

            # print(f'reference\tmeas_value\terror\terror_deriv\terror_sum\tp_result\ti_result\td_result\tpid_result\n')
            # print(f'-------------------------------------------------------------------------------------------------\n')
            # print(f'{pvt_lat} {lat_gps}\t{out_reference}\t{out_meas_value}\t{out_error}\t{out_error_deriv}\t{out_error_sum}\t{out_p_result}\t{out_i_result}\t{out_d_result}\t{out_pid_result}')
            # print(f'{in_reference}\t{in_meas_value}\t{in_error}\t{in_error_deriv}\t{in_error_sum}\t{in_p_result}\t{in_i_result}\t{in_d_result}\t{in_pid_result}')
            # print(f'{in_reference}\t{in_meas_value}\t{in_error}\t{in_error_deriv}\t{in_error_sum}\t{in_p_result}\
            print(motor_one , motor_two , motor_three , motor_four )