# https://www.instructables.com/Benewake-LiDAR-TFmini-Complete-Guide/

import serial
import time

ser = serial.Serial('COM6',115200,timeout = 1)
ser.write(0x42)
ser.write(0x57)
ser.write(0x02)
ser.write(0x00)
ser.write(0x00)
ser.write(0x00)
ser.write(0x01)
ser.write(0x06) # 변경가능

while True:
    while(ser.in_waiting >= 9):
        if('Y' == ser.read() and 'Y' == ser.read()):
            dist_L = ser.read()
            dist_H = ser.read()
            dist_total = ord(dist_H) * 256 + ord(dist_L)
            for i in range(0,5):
                ser.read()
        time.sleep(0.001)
        print(dist_total)