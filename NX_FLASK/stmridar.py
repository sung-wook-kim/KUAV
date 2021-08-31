import os
import serial
import time
import math
import threading
import socket
import numpy as np

global lidar_distance_1 , lidar_distance_2  , mode , q , client_socket

lidar_distance_1 = 0
lidar_distance_2 = 0
mode = 8
q = [5,5,5]
serSTM = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
serSTM.flush()
print("stm")
serLIDAR = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
serLIDAR.flush()
print("lidar")
HOST = '223.171.80.232'
PORT = 9998
print("Waiting GCS")
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(('', PORT))
server_socket.listen()
client_socket, addr = server_socket.accept()
print("connected to GCS")
human_detect = False
img_dx = 0
img_dy = 0
gimbal_plag = False


def connectGCS():
    global mode , q , client_socket
    time.sleep(10)
    plag_1 = False
    plag_6 = False
    plag_9 = False
    while True:
        try:
            print("MODE : " , mode)
            sendingMsg = q.pop(-1)  # sendingmsg = 'mode \n lat_drone \n lon_drone \n gps_time \n lat_person \n lon_person \n altitude \n detection'
            gcs = client_socket.recv(1024).decode().split('\n')
            # print("From GCS : " , gcs)
            if gcs[0] == '1' and plag_1 == False:  # 미션 시작
                mode = 1  # 임무 장소 이동 , 30M 고도 유지
                plag_1 = True
                MISSION_LAT = int(gcs[1]) / 10 ** 7
                MISSION_LON = int(gcs[2]) / 10 ** 7
                plag_MISSION = True
            elif gcs[0] == '6' and plag_6 == False:
                mode = 6  # RTH = 착륙
                plag_6 = True
                RTH_LAT = int(gcs[1]) / 10 ** 7
                RTH_LON = int(gcs[2]) / 10 ** 7
                plag_RTH = True
            # elif gcs[0] == '7' and plag_7 == False:
            #     plag_7 = True
            #     AVOID_LAT = int(gcs[1]) / 10 ** 7
            #     AVOID_LON = int(gcs[2]) / 10 ** 7
                
            elif gcs[0] == '9' and plag_9 == False:
                mode = 9  # 비상 모터 정지
                plag_9 = True
            # print("mode : " ,mode)  # 현재 모드 확인용
            client_socket.sendall(sendingMsg.encode())
        except Exception as e:
            # GCS connection off , connection waiting
            print("connection error" , e)
            client_socket.close()
            client_socket, addr = server_socket.accept()

# 10hz time.sleep(0.1)
def connectLIDAR():
    global lidar_distance_1 , lidar_distance_2
    while True:
        count = serLIDAR.in_waiting
        if count > 8:  # 버퍼에 9바이트 이상 쌓이면
            recv = serLIDAR.read(9)  # read
            serLIDAR.reset_input_buffer()  # 리셋
            if recv[0] == 0x59 and recv[1] == 0x59:  # python3
                lidar_distance_1 = recv[2]  # np.int16(recv[2] + np.int16(recv[3] << 8))
                lidar_distance_2 = recv[3]
                time.sleep(0.1)
                print("LIDAR  : ",lidar_distance_1)
                serLIDAR.reset_input_buffer()

# 5hz because STM transmit is 5hz
# next gps will be calculated when data from stm is received 
def connectSTM():

    global lidar_distance_1 , lidar_distance_2 , q , mode
    header_1 = 0x88
    header_2 = 0x18
    lat_drone = 1;
    lon_drone = 1;
    gps_time = 1;
    roll = 1;
    pitch = 1;
    heading_angle = 1;
    altitude = 1;
    voltage = 1;
    lat_person = 1;
    lon_person = 1

    # Coordination description
    # Front(if heading angle is zero, it is same with North) -> X -> roll direction
    # Left(if heading angle is zero, it is same with East) -> Y -> pitch direction
    # Down(if roll pitch is zero, it is same with Downward) -> Z -> yaw direction

    mode_echo = 0
    lat_prev = 0;
    lon_prev = 0

    while True:
        
        countSTM = serSTM.in_waiting
        if countSTM > 46:
            recvSTM = serSTM.read(47)
            check = 0xffffffff
            for i in range(0,43):
                check -= recvSTM[i]
            serSTM.reset_input_buffer()
            if recvSTM[0] == 0x88 and recvSTM[1] == 0x18:
                mode_echo = np.int16(recvSTM[2])

                lat_drone = np.int32( 
                    np.int32(recvSTM[3] << 56) + np.int32(recvSTM[4] << 48) + np.int32(recvSTM[5] << 40) + np.int32(recvSTM[6] << 32 ) +
                    np.int32(recvSTM[7] << 24) + np.int32(recvSTM[8] << 16) + np.int32(recvSTM[9] << 8) + recvSTM[10]) / 10 ** 7
                lon_drone = np.int32( 
                    np.int32(recvSTM[11] << 56) + np.int32(recvSTM[12] << 48) + np.int32(recvSTM[13] << 40) + np.int32(recvSTM[14] << 32 ) +
                    np.int32(recvSTM[15] << 24) + np.int32(recvSTM[16] << 16) + np.int32(recvSTM[17] << 8) + recvSTM[18]) / 10 ** 7
                gps_time = np.int32(
                    np.int32(recvSTM[19] << 24) + np.int32(recvSTM[20] << 16) + np.int32(recvSTM[21] << 8) +
                    recvSTM[22])

                roll = np.int32(
                    np.int32(recvSTM[23] << 24) + np.int32(recvSTM[24] << 16) + np.int32(recvSTM[25] << 8) +
                    recvSTM[26])
                pitch = np.int32(
                    np.int32(recvSTM[27] << 24) + np.int32(recvSTM[28] << 16) + np.int32(recvSTM[29] << 8) +
                    recvSTM[30])
                heading_angle = np.int32(
                    np.int32(recvSTM[31] << 24) + np.int32(recvSTM[32] << 16) + np.int32(recvSTM[33] << 8) +
                    recvSTM[34])

                altitude = np.int32(
                    np.int32(recvSTM[35] << 24) + np.int32(recvSTM[36] << 16) + np.int32(recvSTM[37] << 8) +
                    recvSTM[38])
                voltage = np.int32(
                    np.int32(recvSTM[39] << 24) + np.int32(recvSTM[40] << 16) + np.int32(recvSTM[41] << 8) +
                    recvSTM[42])
                # checksum = np.int32(
                #     np.int32(recvSTM[43] << 24) + np.int32(recvSTM[44] << 16) + np.int32(recvSTM[45] << 8) +
                #     recvSTM[46])
                checksum_1 = recvSTM[43] & 0xff
                checksum_2 = recvSTM[44] & 0xff
                checksum_3 = recvSTM[45] & 0xff
                checksum_4 = recvSTM[46] & 0xff
                
                checksum = checksum_1 << 24 | checksum_2 << 16 | checksum_3 << 8 | checksum_4
                if checksum == check:

                    print("From STM : " , mode_echo , lat_drone , lon_drone , gps_time , roll , pitch , heading_angle , altitude , voltage)

                serSTM.reset_input_buffer()
            # # 특정 범위에 드론이 들어가면 , AVOID 모드 AVOID on ,off 이므로 확실히 구분 된 조건문
            # # tracking circle + 10m // now 20m
            # if (10 >= haversine.haversine((lat_drone,lon_drone),(AVOID_LAT,AVOID_LON))):
            #     AVOID = True
            # # 벗어나면 , 일반 추적
            # else:
            #     AVOID = False
            #     # 미션 좌표에 도착하면 모드를 2로 변경 ( 그전까지는 1임 )
            # # 0.0000462 -> 2m
            # if (plag_2 == False) and (1 >= haversine.haversine((lat_drone,lon_drone),(MISSION_LAT,MISSION_LON))):  # 10 은 tracking distance인데 다르게 해야할듯
            #     mode = 2  # yaw를 회전하며 탐색 모드
            #     plag_2 = True
            #     plag_MISSION = False

            # # 2번 임무 , 사람이 detect 되지 않았으면 임의의 angle을 통해 회전 
            # if mode == 2 and human_detect == False:
            #     new_gps_lat = lat_drone
            #     new_gps_lon = lon_drone
            #     yaw_error = 20

            #     # 금지구역 모드 X
            # if AVOID == False:
            #     if human_detect == True:
            #         mode = 3  # 추적모드
            #         new_gps_lat = lat_drone + 1  # 계산필요
            #         new_gps_lon = lon_drone + 1  # 계산필요

            #         lat_person = 500000  # 계산필요
            #         lon_person = 500000  # 계산필요

            #         yaw_error = img_dy  # yolo를 통해 인식
            #         # print("mode 3 : " , mode)
            #     else:
            #         mode = 5  # 사람이 추적되지않는 대기모드 일 경우 마지막 추적값을 유지
            #         new_gps_lat = lat_drone if lat_prev == 0 else lat_prev
            #         new_gps_lon = lon_drone if lon_prev == 0 else lon_prev
            #         yaw_error = 0
            #         # print("mode 5 : " , mode)

            # # 금지구역 모드
            # else:
            #     mode = 4
            #     new_gps_lat = lat_drone + 1  # 계산필요
            #     new_gps_lon = lon_drone + 1  # 계산필요

            #     lat_person = 500000  # 계산필요            # time.sleep(0.2)
            #     lon_person = 500000  # 계산필요

            #     yaw_error = img_dy  # yolo를 통해 인식

            new_gps_lat = 371231245
            new_gps_lon = 1271232131
            # ## 1번 , 6번 수행중이라면
            # # 어차피 마지막에 덮어씌우게된다.
            # if plag_MISSION == True:
            #     new_gps_lat = 1
            #     new_gps_lon = 1
            #     yaw_error = 0
            #     mode = 1

            # if plag_RTH == True:
            #     new_gps_lat = RTH_LAT
            #     new_gps_lon = RTH_LON
            #     yaw_error = 0
            #     mode = 6
            # if yaw_error <= 20: # 안전범위 이내라고 판단된다면
            #         yaw_error = 0 # 과도한 조절을 하지 않기 위해 설정
            # 통신을 위한 변경 코드
            lat_prev = new_gps_lat
            lon_prev = new_gps_lon
            new_lat_first = (new_gps_lat >> 24) & 0xff;
            new_lat_second = (new_gps_lat >> 16) & 0xff
            new_lat_third = (new_gps_lat >> 8) & 0xff;
            new_lat_fourth = new_gps_lat & 0xff

            new_lon_first = (new_gps_lon >> 24) & 0xff;
            new_lon_second = (new_gps_lon >> 16) & 0xff
            new_lon_third = (new_gps_lon >> 8) & 0xff;
            new_lon_fourth = new_gps_lon & 0xff

            # mission_lat / mission_lon / roll pitch heading_angle 
            lat_person = lat_drone + 1
            lon_person = lon_drone + 1
            # print(human_detect)
            # print(" calculate next gps ")
            read = str(mode_echo) + '\n' + str(lat_drone) + '\n' + str(lon_drone) + '\n' + str(
                gps_time) + '\n' + str(lat_drone + 1) + '\n' + str(
                lon_drone + 1) + '\n' + str(altitude)
            #print(read)
            q.append(read)
            # 연산 후 바로 next_gps 전달
            yaw_error = 20
            serSTM.write(
                [header_1, header_2, mode, \
                    new_lat_first, new_lat_second, new_lat_third, new_lat_fourth, \
                    new_lon_first, new_lon_second, new_lon_third, new_lon_fourth, \
                    yaw_error, lidar_distance_1, lidar_distance_2])
            
print("thread start")
threadSTM = threading.Thread(target=connectSTM)
threadGCS = threading.Thread(target=connectGCS)
threadLIDAR = threading.Thread(target=connectLIDAR)
threadSTM.start()
threadGCS.start()
threadLIDAR.start()


