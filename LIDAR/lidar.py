import serial
import time
import numpy as np

ser = serial.Serial()
ser.port = 'COM6'    
ser.baudrate = 115200 
ser.open()

# for takeoff / landing

while True:
    count = ser.in_waiting 
    if count > 8: # 버퍼에 9바이트 이상 쌓이면 
        recv = ser.read(9) # read
        ser.reset_input_buffer() # 리셋
        if recv[0] == 0x59 and recv[1] == 0x59:  # python3
            distance = np.int16(recv[2] + np.int16(recv[3] << 8))

            print(f'distance = {distance}cm')
            ser.reset_input_buffer()