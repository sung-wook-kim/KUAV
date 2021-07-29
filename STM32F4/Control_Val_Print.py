import serial
import pandas as pd 
ser = serial.Serial('COM5',115200,timeout=1)
ser.flush()
# 88 , 18
m1_list = []
m2_list = []
m3_list = []
m4_list = []

throttle_li = []
pitch_in_result = []
roll_in_result = []
yaw_heading_result = []
i = 0
alt_li = []

# 77 , 17
out_error_li = []
out_p_result_li = []
out_i_result_li = []
out_d_result_li = []

in_error_li = []
in_p_result_li = []
in_i_result_li = []
in_d_result_li = []

if i % 100 == 0:
    df = pd.DataFrame();
    df['m1'] = m1_list ; df['m2'] = m2_list ; df['m3'] = m3_list ; df['m4'] = m4_list
    df.to_csv("motordata1.csv")
while True:
    i+= 1
    a = int(ser.read(1).hex(),16)
    if a == 0x88:
        b = int(ser.read(1).hex(),16)
        if b == 0x18:    
            motor1_1 = int(ser.read(1).hex() , 16) & 0xff
            motor1_2 = int(ser.read(1).hex() , 16) & 0xff
            motor1_3 = int(ser.read(1).hex() , 16) & 0xff
            motor1_4 = int(ser.read(1).hex() , 16) & 0xff

            motor2_1 = int(ser.read(1).hex() , 16) & 0xff
            motor2_2 = int(ser.read(1).hex() , 16) & 0xff
            motor2_3 = int(ser.read(1).hex() , 16) & 0xff
            motor2_4 = int(ser.read(1).hex() , 16) & 0xff

            motor3_1 = int(ser.read(1).hex() , 16) & 0xff
            motor3_2 = int(ser.read(1).hex() , 16) & 0xff
            motor3_3 = int(ser.read(1).hex() , 16) & 0xff
            motor3_4 = int(ser.read(1).hex() , 16) & 0xff

            motor4_1 = int(ser.read(1).hex() , 16) & 0xff
            motor4_2 = int(ser.read(1).hex() , 16) & 0xff
            motor4_3 = int(ser.read(1).hex() , 16) & 0xff
            motor4_4 = int(ser.read(1).hex() , 16) & 0xff

            throttle_1 = int(ser.read(1).hex() , 16) & 0xff
            throttle_2 = int(ser.read(1).hex() , 16) & 0xff

            pitch_1 = int(ser.read(1).hex() , 16) & 0xff   
            pitch_2 = int(ser.read(1).hex() , 16) & 0xff
            pitch_3 = int(ser.read(1).hex() , 16) & 0xff
            pitch_4 = int(ser.read(1).hex() , 16) & 0xff

            roll_1 = int(ser.read(1).hex() , 16) & 0xff
            roll_2 = int(ser.read(1).hex() , 16) & 0xff
            roll_3 = int(ser.read(1).hex() , 16) & 0xff
            roll_4 = int(ser.read(1).hex() , 16) & 0xff

            yaw_1 = int(ser.read(1).hex() , 16) & 0xff
            yaw_2 = int(ser.read(1).hex() , 16) & 0xff
            yaw_3 = int(ser.read(1).hex() , 16) & 0xff
            yaw_4 = int(ser.read(1).hex() , 16) & 0xff

            alt_1 = int(ser.read(1).hex() , 16) & 0xff
            alt_2 = int(ser.read(1).hex() , 16) & 0xff
            alt_3 = int(ser.read(1).hex() , 16) & 0xff
            alt_4 = int(ser.read(1).hex() , 16) & 0xff

            motor1 = motor1_1 << 24 | motor1_2 << 16 | motor1_3 << 8 | motor1_4
            motor2 = motor2_1 << 24 | motor2_2 << 16 | motor2_3 << 8 | motor2_4
            motor3 = motor3_1 << 24 | motor3_2 << 16 | motor3_3 << 8 | motor3_4
            motor4 = motor4_1 << 24 | motor4_2 << 16 | motor4_3 << 8 | motor4_4

            throttle = throttle_1 << 8 | throttle_2
            pitch = pitch_1 << 24 | pitch_2 << 16 | pitch_3 << 8 | pitch_4
            roll = roll_1 << 24 | roll_2 << 16 | roll_3 << 8 | roll_4
            yaw = yaw_1 << 24 | yaw_2 << 16 | yaw_3 << 8 | yaw_4

            alt = alt_1 << 24 | alt_2 << 16 | alt_3 << 8 | alt_4

            m1_list.append(motor1) ; m2_list.append(motor2) ; m3_list.append(motor3) ; m4_list.append(motor4)
            throttle_li.append(throttle) ; pitch_in_result.append(pitch) ; roll_in_result.append(roll)
            yaw_heading_result.append(yaw) ; alt_li.append(alt)

            pitch_sign = pitch_1 >> 7
            roll_sign = roll_1 >> 7
            yaw_sign = yaw_1 >> 7
            alt_sign = alt_1 >> 7 
            if pitch_sign == 1:
                pitch = (pitch & 0x7fffffff) - 2**31
            if roll_sign == 1:
                roll = (roll & 0x7fffffff) - 2**31
            if yaw_sign == 1:
                yaw = (yaw & 0x7fffffff) - 2**31
            if alt_sign == 1:
                alt = (alt & 0x7fffffff) - 2**31
            print(motor1 , motor2 , motor3 , motor4 , throttle , pitch , roll , yaw ,alt)
    elif a == 0x77:
         b = int(ser.read(1).hex(),16)
         if b == 17:
            out_error_1 = int(ser.read(1).hex() , 16)
            out_error_2 = int(ser.read(1).hex() , 16)
            out_error_3 = int(ser.read(1).hex() , 16)
            out_error_4 = int(ser.read(1).hex() , 16)

            out_p_result_1 = int(ser.read(1).hex() , 16)
            out_p_result_2 = int(ser.read(1).hex() , 16)
            out_p_result_3 = int(ser.read(1).hex() , 16)
            out_p_result_4 = int(ser.read(1).hex() , 16)

            out_i_result_1 = int(ser.read(1).hex() , 16)
            out_i_result_2 = int(ser.read(1).hex() , 16)
            out_i_result_3 = int(ser.read(1).hex() , 16)
            out_i_result_4 = int(ser.read(1).hex() , 16)

            out_d_result_1 = int(ser.read(1).hex() , 16)
            out_d_result_2 = int(ser.read(1).hex() , 16)
            out_d_result_3 = int(ser.read(1).hex() , 16)
            out_d_result_4 = int(ser.read(1).hex() , 16)

            in_error_1 = int(ser.read(1).hex() , 16)
            in_error_2 = int(ser.read(1).hex() , 16)
            in_error_3 = int(ser.read(1).hex() , 16)
            in_error_4 = int(ser.read(1).hex() , 16)

            in_p_result_1 = int(ser.read(1).hex() , 16)
            in_p_result_2 = int(ser.read(1).hex() , 16)
            in_p_result_3 = int(ser.read(1).hex() , 16)
            in_p_result_4 = int(ser.read(1).hex() , 16)

            in_i_result_1 = int(ser.read(1).hex() , 16)
            in_i_result_2 = int(ser.read(1).hex() , 16)
            in_i_result_3 = int(ser.read(1).hex() , 16)
            in_i_result_4 = int(ser.read(1).hex() , 16)

            in_d_result_1 = int(ser.read(1).hex() , 16)
            in_d_result_2 = int(ser.read(1).hex() , 16)
            in_d_result_3 = int(ser.read(1).hex() , 16)
            in_d_result_4 = int(ser.read(1).hex() , 16)
            
            out_error = out_error_1 << 24 | out_error_2 << 16 | out_error_3 << 8 | out_error_4
            out_p_result = out_p_result_1 << 24 | out_p_result_2 << 16 | out_p_result_3 << 8 | out_p_result_4
            out_i_result = out_i_result_1 << 24 | out_i_result_2 << 16 | out_i_result_3 << 8 | out_i_result_4
            out_d_result = out_d_result_1 << 24 | out_d_result_2 << 16 | out_d_result_3 << 8 | out_d_result_4

            in_error = in_error_1 << 24 | in_error_2 << 16 | in_error_3 << 8 | in_error_4
            in_p_result = in_p_result_1 << 24 | in_p_result_2 << 16 | in_p_result_3 << 8 | in_p_result_4
            in_i_result = in_i_result_1 << 24 | in_i_result_2 << 16 | in_i_result_3 << 8 | in_i_result_4
            in_d_result = in_d_result_1 << 24 | in_d_result_2 << 16 | in_d_result_3 << 8 | in_d_result_4
            
            out_error_li.append(out_error)
            out_p_result_li.append(out_p_result)
            out_i_result_li.append(out_i_result)
            out_d_result_li.append(out_d_result)

            in_error_li.append(in_error)
            in_p_result_li.append(in_p_result)
            in_i_result_li.append(in_i_result)
            in_d_result_li.append(in_d_result)
            print(out_error,out_p_result,out_i_result,out_d_result,in_error,in_p_result,in_i_result,in_p_result)
            