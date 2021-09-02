import serial
import time
import pandas as pd
import sys

import keyboard
import threading

#initial Setting
port = 'COM4'
baudrate = 115200

ser = serial.Serial(port, baudrate)
ser.flush()

class Monitor():
    def __init__(self):
        self.header = [0x77, 0x17]

        # Message Protocol
        self.name = ['mode', 'flight_mode', 'failsafe_flag', 'takeoff_step', 'increase_throttle', 'takeoff_throttle',
                     'lat', 'lon', 'lidar', 'baro', 'altitude_setpoint', 'yaw', 'heading_reference', 'throttle', 'batvolt',
                     'lat_setpoint', 'lon_setpoint', 'rth_step', 'decrease_throttle', 'target_yaw', 'gps roll adjust', 'gps pitch adjust' ,'altitude d', 'gps_move factor','lat i in result', 'lon i in result']
        self.byte = [1, 1, 1, 1, 4, 4, 8, 8, 4, 4, 4, 4, 4, 2, 4, 8, 8, 1, 4, 4, 4, 4, 4, 8, 4, 4]
        self.sign = [0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1]
        self.chksum = 0xffffffff
        self.chksum_length = 4

        # Check input
        if not self.is_input_right():
            print('Name & Byte & Sign data length is different')
            sys.exit()

        # Data value
        self.data_length = 0
        for i in range(len(self.byte)):
            self.data_length += self.byte[i]
        self.telemetry_buf = []
        self.new_data = []

        # Save Data
        self.df = pd.DataFrame(columns=self.name)

        # Keyboard input
        self.keyboard = None
        thread = threading.Thread(target=self.read_keyboard)
        thread.start()

    def run(self):
        i = 0
        while True:
            if self.keyboard == 's':
                self.keyboard = None
                now = time.localtime()
                timevar = time.strftime('%d%H%M%S', now)
                self.df.to_csv(f"data/{timevar}_mission_data.csv")
                print('Save Monitor Data')
            elif self.keyboard == 'q':
                self.keyboard = None
                now = time.localtime()
                timevar = time.strftime('%d%H%M%S', now)
                self.df.to_csv(f"data/{timevar}_mission_data.csv")
                print('Save Final Monitor Data')
                sys.exit('quit Monitor')
            elif self.keyboard == 'c':
                self.keyboard = None
                self.df = pd.DataFrame(columns=self.name)
                print('Clear DataFrame')

            if self.is_header_right():
                self.save_in_buf()
                if self.check_data():
                    self.receive_data()
                    self.save_data()

    def save_in_buf(self):
        self.telemetry_buf = []
        for _ in range(self.data_length+self.chksum_length):
            self.telemetry_buf.append(int(ser.read(1).hex(), 16) & 0xff)
        ser.reset_input_buffer()

    def check_data(self):
        self.chksum = 0xffffffff
        self.chksum -= self.header[0]
        self.chksum -= self.header[1]
        received_chksum = self.telemetry_buf[-4] << 24 | self.telemetry_buf[-3] << 16 | self.telemetry_buf[-2] << 8 | self.telemetry_buf[-1]
        for i in range(self.data_length):
            self.chksum -= self.telemetry_buf[i]

        if self.chksum == received_chksum:
            return True
        else:
            return False

    def is_input_right(self):
        if len(self.name) == len(self.byte):
            if len(self.byte) == len(self.sign):
                return 1
        return 0

    def is_header_right(self):
        data = int(ser.read(1).hex(), 16)
        if data == self.header[0]:
            data = int(ser.read(1).hex(), 16)
            if data == self.header[1]:
                return 1
        return 0

    def receive_data(self):
        self.new_data = []
        start = 0
        for i in range(len(self.name)):
            data = 0

            if self.byte[i] == 1:
                data = self.telemetry_buf[start]
            elif self.byte[i] == 2:
                data = self.telemetry_buf[start] << 8 | self.telemetry_buf[start + 1]
            elif self.byte[i] == 3:
                data = self.telemetry_buf[start] << 16 | self.telemetry_buf[start + 1] << 8 | self.telemetry_buf[start + 2]
            elif self.byte[i] == 4:
                data = self.telemetry_buf[start] << 24 | self.telemetry_buf[start + 1] << 16 | self.telemetry_buf[start + 2] << 8 | self.telemetry_buf[start + 3]
            elif self.byte[i] == 8:
                data = self.telemetry_buf[start] << 56 | self.telemetry_buf[start + 1] << 48 | self.telemetry_buf[start + 2] << 40 | self.telemetry_buf[start + 3] << 32 | self.telemetry_buf[start + 4] << 24 | self.telemetry_buf[start + 5] << 16 | \
                       self.telemetry_buf[start + 6] << 8 | self.telemetry_buf[start + 7]

            if self.sign[i]:
                if self.telemetry_buf[start] >> 7:
                    data = (data & 0x7fffffff) - 2 ** 31
            start += self.byte[i]

            self.new_data.append(data)

        for i in range(len(self.name)):
            print(f'{self.name[i]} : {self.new_data[i]}', end='\t')
        print('\n')

    def save_data(self):
        self.df = self.df.append(pd.DataFrame([self.new_data], columns=self.name), ignore_index=True)

    def read_keyboard(self):
        while True:
            self.keyboard = keyboard.read_key()

if __name__ == "__main__" :

    monitor = Monitor()
    monitor.run()