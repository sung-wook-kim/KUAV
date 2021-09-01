import serial
import time
import numpy as np
import struct
ser = serial.Serial('COM12', 115200, timeout=1)
'''
---------------960---------------
Roll Pitch : 5 0 2 40 0 15
Yaw : 35 0.6 0.1 6 0 0.1
altitude : 1500 10 0 1.9 0 0.03
GPS : 1.1   0.01   0.01   0.35  0  0.02
Yaw Rate : 20 0 1.5
'''


# use no arming mode
while True:
    print("1 : 자세 roll, 2 : 자세 pitch , 3 : 자세 yaw , 4 : 고도 , 5 : lat , 1 6 : lon , 7 : Yaw Rate, 8 : 값 확인")
    key = input()
    if key == '1':
        print("자세제어 roll pid 수정")
        a = input().split(' ')
        roll_p_in = int(float(a[0])*100)
        roll_i_in = int(float(a[1])*100)
        roll_d_in = int(float(a[2])*100)
        roll_p_out = int(float(a[3])*100)
        roll_i_out = int(float(a[4])*100)
        roll_d_out = int(float(a[5])*100)

        in_roll_p_1 = (roll_p_in >> 24) & 0xff  
        in_roll_p_2 = (roll_p_in >> 16) & 0xff
        in_roll_p_3 = (roll_p_in >> 8) & 0xff
        in_roll_p_4 = roll_p_in & 0xff

        in_roll_i_1 = (roll_i_in >> 24) & 0xff  
        in_roll_i_2 = (roll_i_in >> 16) & 0xff
        in_roll_i_3 = (roll_i_in >> 8) & 0xff
        in_roll_i_4 =  roll_i_in & 0xff
        
        in_roll_d_1 = (roll_d_in >> 24) & 0xff  
        in_roll_d_2 = (roll_d_in >> 16) & 0xff
        in_roll_d_3 = (roll_d_in >> 8) & 0xff
        in_roll_d_4 =  roll_d_in & 0xff

        out_roll_p_1 = (roll_p_out >> 24) & 0xff  
        out_roll_p_2 = (roll_p_out >> 16) & 0xff
        out_roll_p_3 = (roll_p_out >> 8) & 0xff
        out_roll_p_4 = roll_p_out & 0xff

        out_roll_i_1 = (roll_i_out >> 24) & 0xff  
        out_roll_i_2 = (roll_i_out >> 16) & 0xff
        out_roll_i_3 = (roll_i_out >> 8) & 0xff
        out_roll_i_4 =  roll_i_out & 0xff

        out_roll_d_1 = (roll_d_out >> 24) & 0xff  
        out_roll_d_2 = (roll_d_out >> 16) & 0xff
        out_roll_d_3 = (roll_d_out >> 8) & 0xff
        out_roll_d_4 =  roll_d_out & 0xff

        li_in = [0x46, 0x43, 0x00, in_roll_p_4,  in_roll_p_3,   in_roll_p_2, in_roll_p_1, in_roll_i_4, in_roll_i_3, in_roll_i_2,
                in_roll_i_1, in_roll_d_4, in_roll_d_3, in_roll_d_2, in_roll_d_1, 0x00, 0x00, 0x00, 0x00]
        li_out = [0x46, 0x43, 0x01, out_roll_p_4,  out_roll_p_3, out_roll_p_2, out_roll_p_1, out_roll_i_4, out_roll_i_3, out_roll_i_2,
                out_roll_i_1, out_roll_d_4, out_roll_d_3, out_roll_d_2, out_roll_d_1, 0x00, 0x00, 0x00, 0x00]
        ser.reset_input_buffer()
        for _ in range(3):
            for i in range(5):
                ser.write(li_in)
                # time.sleep(0.2)
            if int(ser.read(1).hex(), 16) == 0x46 and int(ser.read(1).hex(), 16) == 0x43:
                if int(ser.read(1).hex(), 16) == 0x00:
                    drone_roll_in_p = struct.unpack('f',ser.read(4))
                    drone_roll_in_i = struct.unpack('f',ser.read(4))
                    drone_roll_in_d = struct.unpack('f',ser.read(4))
                    print("IN : " , drone_roll_in_p , drone_roll_in_i , drone_roll_in_d )
                    ser.reset_input_buffer()
            # time.sleep(0.2)
            for i in range(5):
                ser.write(li_out)
                # time.sleep(0.2)
            if int(ser.read(1).hex(), 16) == 0x46 and int(ser.read(1).hex(), 16) == 0x43:
                if int(ser.read(1).hex(), 16) == 0x01:
                    drone_roll_out_p = struct.unpack('f',ser.read(4))
                    drone_roll_out_i = struct.unpack('f',ser.read(4))
                    drone_roll_out_d = struct.unpack('f',ser.read(4))
                    print("OUT : " , drone_roll_out_p , drone_roll_out_i , drone_roll_out_d )
                    ser.reset_input_buffer()
            
      
    elif key == '2':
        print("자세제어 pitch pid 수정")
        b = input().split(' ')
        pitch_p_in = int(float(b[0])*100)
        pitch_i_in = int(float(b[1])*100)
        pitch_d_in = int(float(b[2])*100)
        
        pitch_p_out = int(float(b[3])*100)
        pitch_i_out = int(float(b[4])*100)
        pitch_d_out = int(float(b[5])*100)

        in_pitch_p_1 = (pitch_p_in >> 24) & 0xff  
        in_pitch_p_2 = (pitch_p_in >> 16) & 0xff
        in_pitch_p_3 = (pitch_p_in >> 8) & 0xff
        in_pitch_p_4 = pitch_p_in & 0xff

        in_pitch_i_1 = (pitch_i_in >> 24) & 0xff  
        in_pitch_i_2 = (pitch_i_in >> 16) & 0xff
        in_pitch_i_3 = (pitch_i_in >> 8) & 0xff
        in_pitch_i_4 =  pitch_i_in & 0xff
        
        in_pitch_d_1 = (pitch_d_in >> 24) & 0xff  
        in_pitch_d_2 = (pitch_d_in >> 16) & 0xff
        in_pitch_d_3 = (pitch_d_in >> 8) & 0xff
        in_pitch_d_4 =  pitch_d_in & 0xff

        out_pitch_p_1 = (pitch_p_out >> 24) & 0xff  
        out_pitch_p_2 = (pitch_p_out >> 16) & 0xff
        out_pitch_p_3 = (pitch_p_out >> 8) & 0xff
        out_pitch_p_4 = pitch_p_out & 0xff

        out_pitch_i_1 = (pitch_i_out >> 24) & 0xff  
        out_pitch_i_2 = (pitch_i_out >> 16) & 0xff
        out_pitch_i_3 = (pitch_i_out >> 8) & 0xff
        out_pitch_i_4 =  pitch_i_out & 0xff

        out_pitch_d_1 = (pitch_d_out >> 24) & 0xff  
        out_pitch_d_2 = (pitch_d_out >> 16) & 0xff
        out_pitch_d_3 = (pitch_d_out >> 8) & 0xff
        out_pitch_d_4 =  pitch_d_out & 0xff

        li_in = [0x46, 0x43, 0x02, in_pitch_p_4,  in_pitch_p_3,   in_pitch_p_2, in_pitch_p_1, in_pitch_i_4, in_pitch_i_3, in_pitch_i_2,
                in_pitch_i_1, in_pitch_d_4, in_pitch_d_3, in_pitch_d_2, in_pitch_d_1, 0x00, 0x00, 0x00, 0x00]
        li_out = [0x46, 0x43, 0x03, out_pitch_p_4,  out_pitch_p_3, out_pitch_p_2, out_pitch_p_1, out_pitch_i_4, out_pitch_i_3, out_pitch_i_2,
                out_pitch_i_1, out_pitch_d_4, out_pitch_d_3, out_pitch_d_2, out_pitch_d_1, 0x00, 0x00, 0x00, 0x00]
        ser.reset_input_buffer()
        for _ in range(3):
            for i in range(5):
                ser.write(li_in)
                # time.sleep(0.2)
            if int(ser.read(1).hex(), 16) == 0x46 and int(ser.read(1).hex(), 16) == 0x43:
                if int(ser.read(1).hex(), 16) == 0x02:
                    drone_pitch_in_p = struct.unpack('f',ser.read(4))
                    drone_pitch_in_i = struct.unpack('f',ser.read(4))
                    drone_pitch_in_d = struct.unpack('f',ser.read(4))
                    print("IN : " , drone_pitch_in_p , drone_pitch_in_i , drone_pitch_in_d )
                    ser.reset_input_buffer()
            for i in range(5):
                ser.write(li_out)
                # time.sleep(0.2)
            if int(ser.read(1).hex(), 16) == 0x46 and int(ser.read(1).hex(), 16) == 0x43:
                if int(ser.read(1).hex(), 16) == 0x03:
                    drone_pitch_out_p = struct.unpack('f',ser.read(4))
                    drone_pitch_out_i = struct.unpack('f',ser.read(4))
                    drone_pitch_out_d = struct.unpack('f',ser.read(4))
                    print("OUT : " , drone_pitch_out_p , drone_pitch_out_i , drone_pitch_out_d )
                    ser.reset_input_buffer()

    elif key == '3':
        print("자세제어 yaw pid 수정 ")
        c = input().split(' ')
        yaw_p_in = int(float(c[0])*100)
        yaw_i_in = int(float(c[1])*100)
        yaw_d_in = int(float(c[2])*100)
        
        yaw_p_out = int(float(c[3])*100)
        yaw_i_out = int(float(c[4])*100)
        yaw_d_out = int(float(c[5])*100)

        in_yaw_p_1 = (yaw_p_in >> 24) & 0xff  
        in_yaw_p_2 = (yaw_p_in >> 16) & 0xff
        in_yaw_p_3 = (yaw_p_in >> 8) & 0xff
        in_yaw_p_4 = yaw_p_in & 0xff

        in_yaw_i_1 = (yaw_i_in >> 24) & 0xff  
        in_yaw_i_2 = (yaw_i_in >> 16) & 0xff
        in_yaw_i_3 = (yaw_i_in >> 8) & 0xff
        in_yaw_i_4 =  yaw_i_in & 0xff
        
        in_yaw_d_1 = (yaw_d_in >> 24) & 0xff  
        in_yaw_d_2 = (yaw_d_in >> 16) & 0xff
        in_yaw_d_3 = (yaw_d_in >> 8) & 0xff
        in_yaw_d_4 =  yaw_d_in & 0xff

        out_yaw_p_1 = (yaw_p_out >> 24) & 0xff  
        out_yaw_p_2 = (yaw_p_out >> 16) & 0xff
        out_yaw_p_3 = (yaw_p_out >> 8) & 0xff
        out_yaw_p_4 =  yaw_p_out & 0xff

        out_yaw_i_1 = (yaw_i_out >> 24) & 0xff  
        out_yaw_i_2 = (yaw_i_out >> 16) & 0xff
        out_yaw_i_3 = (yaw_i_out >> 8) & 0xff
        out_yaw_i_4 =  yaw_i_out & 0xff

        out_yaw_d_1 = (yaw_d_out >> 24) & 0xff  
        out_yaw_d_2 = (yaw_d_out >> 16) & 0xff
        out_yaw_d_3 = (yaw_d_out >> 8) & 0xff
        out_yaw_d_4 =  yaw_d_out & 0xff

        li_in = [0x46, 0x43, 0x04, in_yaw_p_4,  in_yaw_p_3,   in_yaw_p_2, in_yaw_p_1, in_yaw_i_4, in_yaw_i_3, in_yaw_i_2,
                in_yaw_i_1, in_yaw_d_4, in_yaw_d_3, in_yaw_d_2, in_yaw_d_1, 0x00, 0x00, 0x00, 0x00]
        li_out = [0x46, 0x43, 0x05, out_yaw_p_4,  out_yaw_p_3, out_yaw_p_2, out_yaw_p_1, out_yaw_i_4, out_yaw_i_3, out_yaw_i_2,
                out_yaw_i_1, out_yaw_d_4, out_yaw_d_3, out_yaw_d_2, out_yaw_d_1, 0x00, 0x00, 0x00, 0x00]
        ser.reset_input_buffer()
        for _ in range(3):
            for i in range(5):
                ser.write(li_in)
                #time.sleep(0.2)
            if int(ser.read(1).hex(), 16) == 0x46 and int(ser.read(1).hex(), 16) == 0x43:
                if int(ser.read(1).hex(), 16) == 0x04:
                    drone_yaw_in_p = struct.unpack('f',ser.read(4))
                    drone_yaw_in_i = struct.unpack('f',ser.read(4))
                    drone_yaw_in_d = struct.unpack('f',ser.read(4))
                    print("IN : " , drone_yaw_in_p , drone_yaw_in_i , drone_yaw_in_d )
                    ser.reset_input_buffer()
            for i in range(5):
                ser.write(li_out)
                #time.sleep(0.2)
            if int(ser.read(1).hex(), 16) == 0x46 and int(ser.read(1).hex(), 16) == 0x43:
                if int(ser.read(1).hex(), 16) == 0x05:
                    drone_yaw_out_p = struct.unpack('f',ser.read(4))
                    drone_yaw_out_i = struct.unpack('f',ser.read(4))
                    drone_yaw_out_d = struct.unpack('f',ser.read(4))
                    print("OUT : " , drone_yaw_out_p , drone_yaw_out_i , drone_yaw_out_d )
                    ser.reset_input_buffer()
    elif key == '4':
        print("고도 제어 pid 수정 ")
        d = input().split(' ')
        alt_p_in = int(float(d[0])*100)
        alt_i_in = int(float(d[1])*100)
        alt_d_in = int(float(d[2])*100)
        alt_p_out = int(float(d[3])*100)
        alt_i_out = int(float(d[4])*100)
        alt_d_out = int(float(d[5])*100)

        in_alt_p_1 = (alt_p_in >> 24) & 0xff  
        in_alt_p_2 = (alt_p_in >> 16) & 0xff
        in_alt_p_3 = (alt_p_in >> 8) & 0xff
        in_alt_p_4 =  alt_p_in & 0xff

        in_alt_i_1 = (alt_i_in >> 24) & 0xff  
        in_alt_i_2 = (alt_i_in >> 16) & 0xff
        in_alt_i_3 = (alt_i_in >> 8) & 0xff
        in_alt_i_4 =  alt_i_in & 0xff
        
        in_alt_d_1 = (alt_d_in >> 24) & 0xff  
        in_alt_d_2 = (alt_d_in >> 16) & 0xff
        in_alt_d_3 = (alt_d_in >> 8) & 0xff
        in_alt_d_4 =  alt_d_in & 0xff

        out_alt_p_1 = (alt_p_out >> 24) & 0xff  
        out_alt_p_2 = (alt_p_out >> 16) & 0xff
        out_alt_p_3 = (alt_p_out >> 8) & 0xff
        out_alt_p_4 =  alt_p_out & 0xff

        out_alt_i_1 = (alt_i_out >> 24) & 0xff  
        out_alt_i_2 = (alt_i_out >> 16) & 0xff
        out_alt_i_3 = (alt_i_out >> 8) & 0xff
        out_alt_i_4 =  alt_i_out & 0xff

        out_alt_d_1 = (alt_d_out >> 24) & 0xff  
        out_alt_d_2 = (alt_d_out >> 16) & 0xff
        out_alt_d_3 = (alt_d_out >> 8) & 0xff
        out_alt_d_4 =  alt_d_out & 0xff

        alt_li_in = [0x46, 0x43, 0x06, in_alt_p_4,in_alt_p_3,in_alt_p_2, in_alt_p_1,in_alt_i_4,in_alt_i_3, in_alt_i_2,
                in_alt_i_1, in_alt_d_4, in_alt_d_3, in_alt_d_2, in_alt_d_1, 0x00, 0x00, 0x00, 0x00]
        alt_li_out = [0x46, 0x43, 0x07, out_alt_p_4,  out_alt_p_3, out_alt_p_2, out_alt_p_1, out_alt_i_4, out_alt_i_3, out_alt_i_2,
                out_alt_i_1, out_alt_d_4, out_alt_d_3, out_alt_d_2, out_alt_d_1, 0x00, 0x00, 0x00, 0x00]
        ser.reset_input_buffer()
        for _ in range(3):
            for i in range(5):
                ser.write(alt_li_in)
                #time.sleep(0.2)
            if int(ser.read(1).hex(), 16) == 0x46 and int(ser.read(1).hex(), 16) == 0x43:
                if int(ser.read(1).hex(), 16) == 0x06:
                    drone_alt_in_p = struct.unpack('f',ser.read(4))
                    drone_alt_in_i = struct.unpack('f',ser.read(4))
                    drone_alt_in_d = struct.unpack('f',ser.read(4))
                    print("IN : " , drone_alt_in_p , drone_alt_in_i , drone_alt_in_d )
                    ser.reset_input_buffer()
            for i in range(5):
                ser.write(alt_li_out)
                #time.sleep(0.2)
            if int(ser.read(1).hex(), 16) == 0x46 and int(ser.read(1).hex(), 16) == 0x43:
                if int(ser.read(1).hex(), 16) == 0x07:
                    drone_alt_out_p = struct.unpack('f',ser.read(4))
                    drone_alt_out_i = struct.unpack('f',ser.read(4))
                    drone_alt_out_d = struct.unpack('f',ser.read(4))
                    print("OUT : " , drone_alt_out_p , drone_alt_out_i , drone_alt_out_d )
                    ser.reset_input_buffer()
            
    elif key == '5':
        print("lat제어 PID 수정")
        e = input().split(' ')
        lat_p_in = int(float(e[0])*100)
        lat_i_in = int(float(e[1])*100)
        lat_d_in = int(float(e[2])*100)
        
        lat_p_out = int(float(e[3])*100)
        lat_i_out = int(float(e[4])*100)
        lat_d_out = int(float(e[5])*100)

        in_lat_p_1 = (lat_p_in >> 24) & 0xff  
        in_lat_p_2 = (lat_p_in >> 16) & 0xff
        in_lat_p_3 = (lat_p_in >> 8) & 0xff
        in_lat_p_4 =  lat_p_in & 0xff

        in_lat_i_1 = (lat_i_in >> 24) & 0xff  
        in_lat_i_2 = (lat_i_in >> 16) & 0xff
        in_lat_i_3 = (lat_i_in >> 8) & 0xff
        in_lat_i_4 =  lat_i_in & 0xff
        
        in_lat_d_1 = (lat_d_in >> 24) & 0xff  
        in_lat_d_2 = (lat_d_in >> 16) & 0xff
        in_lat_d_3 = (lat_d_in >> 8) & 0xff
        in_lat_d_4 =  lat_d_in & 0xff

        out_lat_p_1 = (lat_p_out >> 24) & 0xff  
        out_lat_p_2 = (lat_p_out >> 16) & 0xff
        out_lat_p_3 = (lat_p_out >> 8) & 0xff
        out_lat_p_4 =  lat_p_out & 0xff

        out_lat_i_1 = (lat_i_out >> 24) & 0xff  
        out_lat_i_2 = (lat_i_out >> 16) & 0xff
        out_lat_i_3 = (lat_i_out >> 8) & 0xff
        out_lat_i_4 =  lat_i_out & 0xff

        out_lat_d_1 = (lat_d_out >> 24) & 0xff  
        out_lat_d_2 = (lat_d_out >> 16) & 0xff
        out_lat_d_3 = (lat_d_out >> 8) & 0xff
        out_lat_d_4 =  lat_d_out & 0xff

        li_in = [0x46, 0x43, 0x08, in_lat_p_4,  in_lat_p_3,   in_lat_p_2, in_lat_p_1, in_lat_i_4, in_lat_i_3, in_lat_i_2,
                in_lat_i_1, in_lat_d_4, in_lat_d_3, in_lat_d_2, in_lat_d_1, 0x00, 0x00, 0x00, 0x00]
        li_out = [0x46, 0x43, 0x09, out_lat_p_4,  out_lat_p_3, out_lat_p_2, out_lat_p_1, out_lat_i_4, out_lat_i_3, out_lat_i_2,
                out_lat_i_1, out_lat_d_4, out_lat_d_3, out_lat_d_2, out_lat_d_1, 0x00, 0x00, 0x00, 0x00]
        ser.reset_input_buffer()
        for _ in range(3):
            for i in range(5):
                ser.write(li_in)
                #time.sleep(0.2)
            if int(ser.read(1).hex(), 16) == 0x46 and int(ser.read(1).hex(), 16) == 0x43:
                if int(ser.read(1).hex(), 16) == 0x08:
                    drone_lat_in_p = struct.unpack('f',ser.read(4))
                    drone_lat_in_i = struct.unpack('f',ser.read(4))
                    drone_lat_in_d = struct.unpack('f',ser.read(4))
                    print("IN : " , drone_lat_in_p , drone_lat_in_i , drone_lat_in_d )
                    ser.reset_input_buffer()
            for i in range(5):
                ser.write(li_out)
                #time.sleep(0.2)
            if int(ser.read(1).hex(), 16) == 0x46 and int(ser.read(1).hex(), 16) == 0x43:
                if int(ser.read(1).hex(), 16) == 0x09:
                    drone_lat_out_p = struct.unpack('f',ser.read(4))
                    drone_lat_out_i = struct.unpack('f',ser.read(4))
                    drone_lat_out_d = struct.unpack('f',ser.read(4))
                    print("OUT : " , drone_lat_out_p , drone_lat_out_i , drone_lat_out_d )
                    ser.reset_input_buffer()
    elif key == '6':
        print("lon제어 PID 수정")
        e = input().split(' ')
        lon_p_in = int(float(e[0])*100)
        lon_i_in = int(float(e[1])*100)
        lon_d_in = int(float(e[2])*100)
        
        lon_p_out = int(float(e[3])*100)
        lon_i_out = int(float(e[4])*100)
        lon_d_out = int(float(e[5])*100)

        in_lon_p_1 = (lon_p_in >> 24) & 0xff  
        in_lon_p_2 = (lon_p_in >> 16) & 0xff
        in_lon_p_3 = (lon_p_in >> 8) & 0xff
        in_lon_p_4 =  lon_p_in & 0xff

        in_lon_i_1 = (lon_i_in >> 24) & 0xff  
        in_lon_i_2 = (lon_i_in >> 16) & 0xff
        in_lon_i_3 = (lon_i_in >> 8) & 0xff
        in_lon_i_4 =  lon_i_in & 0xff
        
        in_lon_d_1 = (lon_d_in >> 24) & 0xff  
        in_lon_d_2 = (lon_d_in >> 16) & 0xff
        in_lon_d_3 = (lon_d_in >> 8) & 0xff
        in_lon_d_4 =  lon_d_in & 0xff

        out_lon_p_1 = (lon_p_out >> 24) & 0xff  
        out_lon_p_2 = (lon_p_out >> 16) & 0xff
        out_lon_p_3 = (lon_p_out >> 8) & 0xff
        out_lon_p_4 =  lon_p_out & 0xff

        out_lon_i_1 = (lon_i_out >> 24) & 0xff  
        out_lon_i_2 = (lon_i_out >> 16) & 0xff
        out_lon_i_3 = (lon_i_out >> 8) & 0xff
        out_lon_i_4 =  lon_i_out & 0xff

        out_lon_d_1 = (lon_d_out >> 24) & 0xff  
        out_lon_d_2 = (lon_d_out >> 16) & 0xff
        out_lon_d_3 = (lon_d_out >> 8) & 0xff
        out_lon_d_4 =  lon_d_out & 0xff

        li_in = [0x46, 0x43, 0x0a, in_lon_p_4,  in_lon_p_3,   in_lon_p_2, in_lon_p_1, in_lon_i_4, in_lon_i_3, in_lon_i_2,
                in_lon_i_1, in_lon_d_4, in_lon_d_3, in_lon_d_2, in_lon_d_1, 0x00, 0x00, 0x00, 0x00]
        li_out = [0x46, 0x43, 0x0b, out_lon_p_4,  out_lon_p_3, out_lon_p_2, out_lon_p_1, out_lon_i_4, out_lon_i_3, out_lon_i_2,
                out_lon_i_1, out_lon_d_4, out_lon_d_3, out_lon_d_2, out_lon_d_1, 0x00, 0x00, 0x00, 0x00]
        ser.reset_input_buffer()
        for _ in range(3):
            for i in range(5):
                ser.write(li_in)
                #time.sleep(0.2)
            if int(ser.read(1).hex(), 16) == 0x46 and int(ser.read(1).hex(), 16) == 0x43:
                if int(ser.read(1).hex(), 16) == 0x0a:
                    drone_lon_in_p = struct.unpack('f',ser.read(4))
                    drone_lon_in_i = struct.unpack('f',ser.read(4))
                    drone_lon_in_d = struct.unpack('f',ser.read(4))
                    print("IN : " , drone_lon_in_p , drone_lon_in_i , drone_lon_in_d )
                    ser.reset_input_buffer()
            for i in range(5):
                ser.write(li_out)
                #time.sleep(0.2)
            if int(ser.read(1).hex(), 16) == 0x46 and int(ser.read(1).hex(), 16) == 0x43:
                if int(ser.read(1).hex(), 16) == 0x0b:
                    drone_lon_out_p = struct.unpack('f',ser.read(4))
                    drone_lon_out_i = struct.unpack('f',ser.read(4))
                    drone_lon_out_d = struct.unpack('f',ser.read(4))
                    print("OUT : " , drone_lon_out_p , drone_lon_out_i , drone_lon_out_d )
                    ser.reset_input_buffer()
    elif key == '7':
        print("yaw rate gain 입력")
        e = input().split(' ')
        yaw_rate_p_in = int(float(e[0])*100)
        yaw_rate_i_in = int(float(e[1])*100)
        yaw_rate_d_in = int(float(e[2])*100)
        
        yaw_rate_p_1 = (yaw_rate_p_in >> 24) & 0xff  
        yaw_rate_p_2 = (yaw_rate_p_in >> 16) & 0xff
        yaw_rate_p_3 = (yaw_rate_p_in >> 8) & 0xff
        yaw_rate_p_4 =  yaw_rate_p_in & 0xff

        yaw_rate_i_1 = (yaw_rate_i_in >> 24) & 0xff  
        yaw_rate_i_2 = (yaw_rate_i_in >> 16) & 0xff
        yaw_rate_i_3 = (yaw_rate_i_in >> 8) & 0xff
        yaw_rate_i_4 =  yaw_rate_i_in & 0xff
        
        yaw_rate_d_1 = (yaw_rate_d_in >> 24) & 0xff  
        yaw_rate_d_2 = (yaw_rate_d_in >> 16) & 0xff
        yaw_rate_d_3 = (yaw_rate_d_in >> 8) & 0xff
        yaw_rate_d_4 =  yaw_rate_d_in & 0xff

        li_in = [0x46, 0x43, 0x0c, yaw_rate_p_4,  yaw_rate_p_3,   yaw_rate_p_2, yaw_rate_p_1, yaw_rate_i_4, yaw_rate_i_3, yaw_rate_i_2,
                yaw_rate_i_1, yaw_rate_d_4, yaw_rate_d_3, yaw_rate_d_2, yaw_rate_d_1, 0x00, 0x00, 0x00, 0x00]
        ser.reset_input_buffer()
        for _ in range(3):
            for i in range(5):
                ser.write(li_in)
                #time.sleep(0.2)
            if int(ser.read(1).hex(), 16) == 0x46 and int(ser.read(1).hex(), 16) == 0x43:
                if int(ser.read(1).hex(), 16) == 0x0c:
                    yaw_rate_in_p = struct.unpack('f',ser.read(4))
                    yaw_rate_in_i = struct.unpack('f',ser.read(4))
                    yaw_rate_in_d = struct.unpack('f',ser.read(4))
                    print("YAW RATE : " , yaw_rate_in_p , yaw_rate_in_i , yaw_rate_in_d )
                    ser.reset_input_buffer()
        
    elif key == '8':
        print("gain 확인")
        li_in = [0x46, 0x43, 0x0d]
            # , in_lon_p_4,  in_lon_p_3,   in_lon_p_2, in_lon_p_1, in_lon_i_4, in_lon_i_3, in_lon_i_2,
            #     in_lon_i_1, in_lon_d_4, in_lon_d_3, in_lon_d_2, in_lon_d_1, 0x00, 0x00, 0x00, 0x00]
        for _ in range(3):
            for i in range(5):
                ser.write(li_in)
            if int(ser.read(1).hex(), 16) == 0x46 and int(ser.read(1).hex(), 16) == 0x43:
                if int(ser.read(1).hex(), 16) == 0x0:
                    roll_in_p = struct.unpack('f',ser.read(4))
                    roll_in_i = struct.unpack('f',ser.read(4))
                    roll_in_d = struct.unpack('f',ser.read(4))
                    roll_out_p = struct.unpack('f',ser.read(4))
                    roll_out_i = struct.unpack('f',ser.read(4))
                    roll_out_d = struct.unpack('f',ser.read(4))
                    pitch_in_p = struct.unpack('f',ser.read(4))
                    pitch_in_i = struct.unpack('f',ser.read(4))
                    pitch_in_d = struct.unpack('f',ser.read(4))
                    pitch_out_p = struct.unpack('f',ser.read(4))
                    pitch_out_i = struct.unpack('f',ser.read(4))
                    pitch_out_d = struct.unpack('f',ser.read(4))
                    yaw_in_p = struct.unpack('f',ser.read(4))
                    yaw_in_i = struct.unpack('f',ser.read(4))
                    yaw_in_d = struct.unpack('f',ser.read(4))
                    yaw_out_p = struct.unpack('f',ser.read(4))
                    yaw_out_i = struct.unpack('f',ser.read(4))
                    yaw_out_d = struct.unpack('f',ser.read(4))
                    alt_in_p = struct.unpack('f',ser.read(4))
                    alt_in_i = struct.unpack('f',ser.read(4))
                    alt_in_d = struct.unpack('f',ser.read(4))
                    alt_out_p = struct.unpack('f',ser.read(4))
                    alt_out_i = struct.unpack('f',ser.read(4))
                    alt_out_d = struct.unpack('f',ser.read(4))
                    lat_in_p = struct.unpack('f',ser.read(4))
                    lat_in_i = struct.unpack('f',ser.read(4))
                    lat_in_d = struct.unpack('f',ser.read(4))
                    lat_out_p = struct.unpack('f',ser.read(4))
                    lat_out_i = struct.unpack('f',ser.read(4))
                    lat_out_d = struct.unpack('f',ser.read(4))
                    lon_in_p = struct.unpack('f',ser.read(4))
                    lon_in_i = struct.unpack('f',ser.read(4))
                    lon_in_d = struct.unpack('f',ser.read(4))
                    lon_out_p = struct.unpack('f',ser.read(4))
                    lon_out_i = struct.unpack('f',ser.read(4))
                    lon_out_d = struct.unpack('f',ser.read(4))
                    
                    print("자세 roll in : " , roll_in_p , roll_in_i , roll_in_d )
                    print("자세 roll out : " , roll_out_p , roll_out_i , roll_out_d )
                    print("자세 pitch in : " , pitch_in_p , pitch_in_i , pitch_in_d )
                    print("자세 pitch out : " , pitch_out_p , pitch_out_i , pitch_out_d )
                    print("자세 yaw in : " , yaw_in_p , yaw_in_i , yaw_in_d )
                    print("자세 yaw out : " , yaw_out_p , yaw_out_i , yaw_out_d )
                    print("고도 in : " , alt_in_p , alt_in_i , alt_in_d)
                    print("고도 out : " , alt_out_p , alt_out_i , alt_out_d)
                    print("gps lat in : " , lat_in_p , lat_in_i , lat_in_d)
                    print("gps lat out : " , lat_out_p , lat_out_i , lat_out_d)
                    print("gps lon in : " , lon_in_p , lon_in_i , lon_in_d)
                    print("gps lon out : " , lon_out_p , lon_out_i , lon_out_d)
                    ser.reset_input_buffer()        

            
 #   input("enter the PID gain : ")
