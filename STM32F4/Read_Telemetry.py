import serial

ser = serial.Serial('COM7' ,115200)

a = int(ser.read(1).hex(),16)

while(1):
    if a == 0x88:
        b = int(ser.read(1).hex(),16)
        if b == 0x18:    
            alt_1 = int(ser.read(1).hex() , 16) & 0xff
            alt_sign = alt_1 >> 7
            alt_2 = int(ser.read(1).hex() , 16) & 0xff
            alt_3 = int(ser.read(1).hex() , 16) & 0xff
            alt_4 = int(ser.read(1).hex() , 16) & 0xff

            LV_1 = int(ser.read(1).hex() , 16) & 0xff
            LV_2 = int(ser.read(1).hex() , 16) & 0xff

            alt = alt_1 << 24 | alt_2 << 16 | alt_3 << 8 | alt_4
            if alt_sign == 1: (alt & 0x7fffffff) - 2**31 
            LV = LV_1 << 8 | LV_2

    print("actual_pressure_fast : ", alt)
    print("LV : ", LV)

