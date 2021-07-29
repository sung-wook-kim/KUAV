import serial 
import time
import math
import threading
import socket

import pandas as pd
class NX():
    def __init__(self):
        self.MISSION_LAT = 32000000
        self.MISSION_LON = 31000000
        self.mode = 0 # default = 0
        self.plag_1 = False ; self.plag_2 = False ; self.plag_3 = False
        self.plag_4 = False ; self.plag_5 = False ; self.plag_6 = False
        self.plag_9 = False
        ## flag for MISSION
        
        ## if GPS AVOID CONDITION ==> self.AVOID = True
        self.AVOID = False ; self.HIDE = False ; self.MIDPOINT = False 
        self.ground = [0,0,0]
        # NX - GCS socket , NX = server
        self.HOST = '192.168.43.185'  
        self.PORT = 9999
        print("waiting")
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind(('', self.PORT))
        self.server_socket.listen()
        self.client_socket, self.addr = self.server_socket.accept()
        
        self.thread1 = threading.Thread(target = self.connectSTM)
        self.thread2 = threading.Thread(target = self.connectGCS)
        self.thread1.start()
        self.thread2.start()

    def connectGCS(self):
        while True:
            print("GCS")
            time.sleep(1)
            sendingMsg = self.ground[-1]# sendingmsg = 'mode \n lat_drone \n lon_drone \n lat_person \n lon_person \n altitude'
            recv = self.client_socket.recv(1024).decode().split('\n')
            plag = recv[0]
            if plag == '1' and self.plag_1 == False:
                self.mode = 1 # 임무 장소 이동 , 30M 고도 유지
                self.plag_1 = True
                self.MISSION_LAT = float(recv[1])
                self.MISSION_LON = float(recv[2])

            elif plag == '6' and self.plag_6 == False :
                self.mode = 6 # RTH = 착륙
                self.plag_6 = True
            elif plag == '9' and self.plag_9 == False:
                self.mode = 9 # 비상 모터 정지
                self.plag_9 = True
            print(self.mode) # 현재 모드 확인용 
            self.client_socket.sendall(sendingMsg.encode())

    def connectSTM(self):
        i = 0
        header_1 = 0x44
        header_2 = 0x77
        while True:
            i+=1
            print("STM")

            temp = [1 , 370001000, 38000000 , 120 , 130 , 150 , 200]
            mode_echo = temp[0] ; lat_drone = temp[1] ; lon_drone = temp[2] 
            roll = temp[3] ; pitch = temp[4] ; heading_angle = temp[5] ; altitude = temp[6]
            #if time.time() - start >= 3 and plag_2 == False:
            #    mode = 2 # yaw를 회전하며 탐색 모드
            #    plag_2 = True
            if  self.plag_2 == False and self.MISSION_LAT == lat_drone and self.MISSION_LON == lon_drone:
                self.mode = 2 # yaw를 회전하며 탐색 모드
                self.plag_2 = True

            elif self.plag_3 == False and self.MIDPOINT == True and self.AVOID == False:
                self.mode = 3 # 추적 모드
                self.plag_3 = True

            elif self.plag_4 == False and self.AVOID == True:
                self.mode = 4 # 금지 구역 모드
                self.plag_4 = True 
                self.plag_3 = False
            elif  self.plag_5 == False  and self.HIDE == True :
                self.mode = 5 # 임무 대기 모드 ( 파라솔 )
                self.plag_5 = True 
                self.plag_3 = False # 추적 모드 재 가동을 위한 플래그 off

            ## new_gps code , person gps _code
            new_gps_lat = lat_drone +1 ; new_gps_lon = lon_drone +1 ; yaw_error = 500000000
            lat_person = 500000 ; lon_person = 500000

            new_lat_first = (new_gps_lat >> 24) & 0xff ; new_lat_second =  (new_gps_lat >> 16) & 0xff
            new_lat_third =  (new_gps_lat >> 8) & 0xff ; new_lat_fourth =  new_gps_lat  & 0xff

            new_lon_first = (new_gps_lon >> 24) & 0xff ; new_lon_second =  (new_gps_lon >> 16) & 0xff
            new_lon_third =  (new_gps_lon >> 8) & 0xff ; new_lon_fourth =  new_gps_lon & 0xff
            yaw_error = 0x05

            read = str(self.mode)+'\n'+str(lat_drone)+'\n'+str(lon_drone)+'\n'+str(lat_person)+'\n'+str(lon_person)+'\n'+str(altitude)
            self.ground.append(read)
            time.sleep(0.5)
            if i % 111 == 0:
                # ser.write(
                #     [header_1,header_2,mode,\
                #     new_lat_first,new_lat_second,new_lat_third,new_lat_fourth,\
                #     new_lon_first,new_lon_second,new_lon_third,new_lon_fourth,\
                #     yaw_error]
                # )
                pass
                #df = pd.DataFrame()
                #df['gps_lat'] = gps_lat
                #df['gps_lon'] = gps_lon
                #df['alt'] = alt
                #df.to_csv("GPSdata.csv")
