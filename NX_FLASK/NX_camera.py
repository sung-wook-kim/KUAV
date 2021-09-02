import os
import serial
import time
import math
import threading
import socket

import cv2
from base_camera import BaseCamera
import torch
import torch.nn as nn
import torchvision
import numpy as np
import argparse
import haversine
from utils.datasets import *
from utils.utils import *
from storm32 import Storm32
from util import *

NO_FLY_RADIUS = 20  # 10m


class NX(BaseCamera):
    video_source = 'test.mp4'

    def __init__(self):
        self.serSTM = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
        self.serSTM.flush()
        # stm에서 받아올 변수 전부 self로 저장후 사용
        # Drone data
        self.drone_data = DroneData()
        self.drone_temp = DroneData()
        # Estimation
        self.lat_person = None
        self.lon_person = None
        print("stm")

        self.serLIDAR = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        self.serLIDAR.flush()
        print("lidar")

        # Gimbal start
        self.gimbal = Storm32(port='/dev/ttyACM0')
        self.g_pitch, self.g_roll, _ = self.gimbal.get_imu1_angles()
        print("gimbal")

        # Coord estimator
        self.coord_est = CoordEstimator()

        # Mission params
        self.hovering_altitude = -30  # meters
        self.default_gimbal_pitch = -45  # degrees, how much drone will look down at spawn
        self.following_distance = self.hovering_altitude / math.tan(self.default_gimbal_pitch)

        self.lidar_distance_1 = 0
        self.lidar_distance_2 = 0
        self.MISSION_LAT = 365804666,  # 사람 출발 위치
        self.MISSION_LON = 127526113
        self.plag_MISSION = False
        self.plag_RTH = False
        self.AVOID_LAT = 365813333;
        self.AVOID_LON = 127526319
        self.mode = 0  # default = 0
        self.plag_1 = False
        self.plag_2 = False
        self.plag_6 = False
        self.plag_9 = False
        self.AVOID = False
        self.q = [0, 0, 0]
        # NX - GCS socket , NX = server
        # self.HOST = '223.171.80.232'
        self.HOST = '172.20.10.2'
        self.PORT = 9998
        print("Waiting GCS")
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind(('', self.PORT))
        self.server_socket.listen()
        self.client_socket, self.addr = self.server_socket.accept()
        print("connected tp GCS")
        self.human_detect = False
        self.img_dx = 0
        self.img_dy = 0
        self.gimbal_plag = False

        self.threadSTM = threading.Thread(target=self.connectSTM, daemon=True)
        self.threadGCS = threading.Thread(target=self.connectGCS, daemon=True)
        self.threadLIDAR = threading.Thread(target=self.connectLIDAR, daemon=True)
        self.threadGIMBAL = threading.Thread(target=self.connectGIMBAL, daemon=True)
        self.threadSTM.start()
        self.threadGCS.start()
        self.threadLIDAR.start()
        self.threadGIMBAL.start()
        super(NX, self).__init__()

    def frames(self):
        out, weights, imgsz, stride = \
            'inference/output', './visdrone_trained_model/weights/best.pt', 640, 32
        # source = 'test.mp4'
        source = 0
        device = torch_utils.select_device()

        if os.path.exists(out):
            shutil.rmtree(out)  # delete output folder
        os.makedirs(out)  # make new output folder

        # Load model
        google_utils.attempt_download(weights)
        model = torch.load(weights, map_location=device)['model']
        model.to(device).eval()

        # Half precision
        # half = False and device.type != 'cpu'
        half = True and device.type != 'cpu'
        print('half = ' + str(half))
        if half:
            model.half()
        dataset = LoadStreams(source, img_size=imgsz)
        names = model.names if hasattr(model, 'names') else model.modules.names
        colors = [[random.randint(0, 255) for _ in range(3)] for _ in range(len(names))]

        # Run inference
        t0 = time.time()
        img = torch.zeros((1, 3, imgsz, imgsz), device=device)  # init img
        _ = model(img.half() if half else img) if device.type != 'cpu' else None  # run once

        # to check the detection
        self.human_detect = False  # if the model do detect
        w, h = int(dataset.imgs[0].shape[1]), int(dataset.imgs[0].shape[0])
        self.img_human_center = (int(w / 2), int(h / 2))
        self.img_human_foot = self.img_human_center
        self.img_dx = 0
        self.img_dy = 0
        n = 0  # the number of detect iteration
        conf_thres = 0.4
        fontFace = cv2.FONT_HERSHEY_SIMPLEX
        fontScale = 0.5
        thickness = 2
        textSize, baseline = cv2.getTextSize("FPS", fontFace, fontScale, thickness)

        for path, img, im0s, vid_cap in dataset:

            if (self.mode == 0) or (self.mode == 1) or (self.mode == 6):
                self.H_x, self._H_y = self.prev_H_x, self.prev_H_y
                yield cv2.imencode('.jpg', im0)[1].tobytes()
                continue

            self.drone_temp = self.drone_data

            img = torch.from_numpy(img).to(device)
            img = img.half() if half else img.float()  # uint8 to fp16/32
            img /= 255.0  # 0 - 255 to 0.0 - 1.0
            if img.ndimension() == 3:
                img = img.unsqueeze(0)

            # Inference
            t1 = torch_utils.time_synchronized()
            pred_raw = model(img, augment=False)[0]
            # Apply NMS
            pred = non_max_suppression(pred_raw, conf_thres=conf_thres, iou_thres=0.5, fast=True, classes=None,
                                       agnostic=False)
            t2 = torch_utils.time_synchronized()

            for i, det in enumerate(pred):  # detections per image
                p, s, im0 = path[i], f'{i}: ', im0s[i].copy()

                s += '%gx%g ' % img.shape[2:]  # print string
                if det is not None and len(det):
                    # Rescale boxes from img_size to im0 size
                    det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

                    # for c in det[:, -1].unique():  #probably error with torch 1.5
                    for c in det[:, -1].detach().unique():
                        n = (det[:, -1] == c).sum()  # detections per class
                        s += '%g %s, ' % (n, names[int(c)])  # add to string

                    self.human_detect = False
                    min_len = 10
                    t_cal1 = time.time()
                    self.prev_H_x, self.prev_H_y = self.H_x, self.H_y
                    for *xyxy, conf, cls in det:
                        label = '%s %.2f' % (names[int(cls)], conf)
                        center, foot = plot_one_box2(xyxy, im0, label=label, color=colors[int(cls)],
                                                     line_thickness=1)
                        if int(cls) == 0:
                            H_x, H_y = CoordEstimator.est(drone_data=self.drone_data, foot=foot)
                            foot_len = math.sqrt((H_x - self.prev_H_x) ** 2 + (H_y - self.prev_H_y) ** 2)
                            if min_len > foot_len:
                                min_len = foot_len
                                self.img_human_center = center
                                self.img_human_foot = foot
                                self.H_x, self.H_y = H_x, H_y
                                self.human_detect = True
                                self.img_dx = self.img_human_foot[0] - w / 2
                                self.img_dy = self.img_human_foot[1] - h / 2
                                self.gimbal_plag = True
                    t_cal2 = time.time()
                    print('coord_est_time', t_cal2 - t_cal1)
                    if self.human_detect:
                        cv2.line(im0,
                                 (int(self.img_dx + im0.shape[1] / 2), int(im0.shape[0] / 2)),
                                 (int(im0.shape[1] / 2), int(im0.shape[0] / 2)), (0, 0, 255),
                                 thickness=1, lineType=cv2.LINE_AA)
                        cv2.line(im0,
                                 (int(im0.shape[1] / 2), int(self.img_dy + im0.shape[0] / 2)),
                                 (int(im0.shape[1] / 2), int(im0.shape[0] / 2)), (0, 0, 255),
                                 thickness=1, lineType=cv2.LINE_AA)
                        cv2.putText(im0, 'Human Detection ' + str(self.human_detect), (10, 10 + textSize[1]), fontFace,
                                    fontScale, (255, 0, 255), thickness)
                        cv2.putText(im0, str(f"{self.H_x:.2f}, {self.H_y:.2f}"),
                                    (self.img_human_foot[0] - 30, self.img_human_foot[1]), fontFace,
                                    fontScale, (255, 0, 255), thickness)
                print('%sDone. (%.3fs)' % (s, t2 - t1), self.img_human_center)
            yield cv2.imencode('.jpg', im0)[1].tobytes()

    def connectGIMBAL(self):
        target_pitch = 0
        while True:
            if self.gimbal_plag == True:
                self.gimbal_plag = False
                pitch_gain = 0.01
                target_pitch += round((pitch_gain * self.img_dy), 2)
                #   Pitch     Roll      Yaw      axislimit
                # [Min, Max, Min, Max, Min, Max]
                print(f"img_dx {self.img_dx}, img_dy {self.img_dy}, target_pitch {target_pitch}")
                self.gimbal.set_angles(target_pitch)
                self.drone_data.g_pitch, self.drone_data.g_roll, _ = self.gimbal.get_imu1_angles()
            else:
                time.sleep(0.01)

    # 1hz because of gcs -> update_gps 
    def connectGCS(self):
        time.sleep(10)
        while True:
            try:
                sendingMsg = self.q.pop(-1)
                # sendingmsg = 'mode \n lat_drone \n lon_drone \n gps_time \n lat_person \n lon_person \n altitude \n detection'
                gcs = self.client_socket.recv(1024).decode().split('\n')
                # print("From GCS : " , gcs)
                if gcs[0] == '1' and self.plag_1 == False:  # 미션 시작
                    self.mode = 1  # 임무 장소 이동 , 30M 고도 유지
                    self.plag_1 = True
                    self.MISSION_LAT = int(gcs[1]) / 10 ** 7
                    self.MISSION_LON = int(gcs[2]) / 10 ** 7
                    self.plag_MISSION = True

                elif gcs[0] == '6' and self.plag_6 == False:
                    self.mode = 6  # RTH = 착륙
                    self.plag_6 = True
                    self.RTH_LAT = int(gcs[1]) / 10 ** 7  # 365804861
                    self.RTH_LON = int(gcs[2]) / 10 ** 7  # 1275257444
                    self.plag_RTH = True

                elif gcs[0] == '7' and self.plag_7 == False:
                    self.plag_7 = True
                    self.AVOID_LAT = int(gcs[1]) / 10 ** 7
                    self.AVOID_LON = int(gcs[2]) / 10 ** 7

                elif gcs[0] == '9' and self.plag_9 == False:
                    self.mode = 9  # 비상 모터 정지
                    self.plag_9 = True
                # print("mode : " ,self.mode)  # 현재 모드 확인용
                self.client_socket.sendall(sendingMsg.encode())
            except Exception as e:
                # GCS connection off , connection waiting
                print("connection error")
                self.client_socket.close()
                self.client_socket, self.addr = self.server_socket.accept()

    # 10hz time.sleep(0.1)
    def connectLIDAR(self):
        while True:
            count = self.serLIDAR.in_waiting
            if count > 8:  # 버퍼에 9바이트 이상 쌓이면
                recv = self.serLIDAR.read(9)  # read
                self.serLIDAR.reset_input_buffer()  # 리셋
                if recv[0] == 0x59 and recv[1] == 0x59:  # python3
                    real_lidar = np.int16(recv[2] + np.int16(recv[3] << 8))
                    lidar_adjust = int(real_lidar * math.cos(self.drone_data.roll * 0.017454) * math.cos(
                        self.drone_data.pitch * 0.017454))
                    self.lidar_distance_1 = (lidar_adjust >> 8) & 0xff
                    self.lidar_distance_2 = lidar_adjust & 0xff
                    # lidar_distance_1 = recv[2]  # np.int16(recv[2] + np.int16(recv[3] << 8))
                    # lidar_distance_2 = recv[3]
                    time.sleep(0.1)
                    print("LIDAR  : ", lidar_adjust)
                    self.serLIDAR.reset_input_buffer()

    # 5hz because STM transmit is 5hz
    # next gps will be calculated when data from stm is received 
    def connectSTM(self):
        header_1 = 0x88;
        header_2 = 0x18
        gps_time = 1
        # Coordination description
        # Front(if heading angle is zero, it is same with North) -> X -> roll direction
        # Left(if heading angle is zero, it is same with East) -> Y -> pitch direction
        # Down(if roll pitch is zero, it is same with Downward) -> Z -> yaw direction
        new_gps_lat = 0
        new_gps_lon = 0
        start_target_yaw = 0
        mode2_iter = 0
        mode2_yaw_dir = 1
        mode_echo = 0
        lat_prev = 0
        lon_prev = 0

        while True:
            countSTM = self.serSTM.in_waiting
            if countSTM > 46:
                recvSTM = self.serSTM.read(47)
                check = 0xffffffff
                for i in range(0, 43):
                    check -= recvSTM[i]
                self.serSTM.reset_input_buffer()
                if recvSTM[0] == 0x88 and recvSTM[1] == 0x18:
                    mode_echo = np.int16(recvSTM[2])
                    self.drone_data.lat_drone = np.int64(
                        np.int32(recvSTM[3] << 56) + np.int32(recvSTM[4] << 48) + np.int32(recvSTM[5] << 40) + np.int32(
                            recvSTM[6] << 32) +
                        np.int32(recvSTM[7] << 24) + np.int32(recvSTM[8] << 16) + np.int32(recvSTM[9] << 8) + recvSTM[
                            10]) / 10 ** 7
                    self.drone_data.lon_drone = np.int64(
                        np.int32(recvSTM[11] << 56) + np.int32(recvSTM[12] << 48) + np.int32(
                            recvSTM[13] << 40) + np.int32(
                            recvSTM[14] << 32) +
                        np.int32(recvSTM[15] << 24) + np.int32(recvSTM[16] << 16) + np.int32(recvSTM[17] << 8) +
                        recvSTM[
                            18]) / 10 ** 7
                    gps_time = np.int32(
                        np.int32(recvSTM[19] << 24) + np.int32(recvSTM[20] << 16) + np.int32(recvSTM[21] << 8) +
                        recvSTM[22])

                    self.drone_data.roll = np.int32(
                        np.int32(recvSTM[23] << 24) + np.int32(recvSTM[24] << 16) + np.int32(recvSTM[25] << 8) +
                        recvSTM[26])
                    self.drone_data.pitch = np.int32(
                        np.int32(recvSTM[27] << 24) + np.int32(recvSTM[28] << 16) + np.int32(recvSTM[29] << 8) +
                        recvSTM[30])
                    self.drone_data.heading_angle = np.int32(
                        np.int32(recvSTM[31] << 24) + np.int32(recvSTM[32] << 16) + np.int32(recvSTM[33] << 8) +
                        recvSTM[34])

                    self.drone_data.altitude = np.int32(
                        np.int32(recvSTM[35] << 24) + np.int32(recvSTM[36] << 16) + np.int32(recvSTM[37] << 8) +
                        recvSTM[38])
                    voltage = np.int32(
                        np.int32(recvSTM[39] << 24) + np.int32(recvSTM[40] << 16) + np.int32(recvSTM[41] << 8) +
                        recvSTM[42])
                    # print("From STM : " , mode_echo , lat_drone , lon_drone , gps_time , roll , pitch , heading_angle , altitude , voltage)
                    checksum_1 = recvSTM[43] & 0xff
                    checksum_2 = recvSTM[44] & 0xff
                    checksum_3 = recvSTM[45] & 0xff
                    checksum_4 = recvSTM[46] & 0xff
                    checksum = checksum_1 << 24 | checksum_2 << 16 | checksum_3 << 8 | checksum_4
                    if checksum == check:
                        print("From STM : ", mode_echo, self.drone_data.lat_drone, self.drone_data.lon_drone, gps_time,
                              self.drone_data.roll,
                              self.drone_data.pitch, self.drone_data.heading_angle, self.drone_data.altitude, voltage)

                    self.serSTM.reset_input_buffer()
                    self.drone_data.D_x, self.drone_data.D_y = get_metres_location(self.drone_data.home_lat_lon,
                                                                         [self.drone_data.lat_drone, self.drone_data.lon_drone])
                # 특정 범위에 드론이 들어가면 , AVOID 모드 AVOID on ,off 이므로 확실히 구분 된 조건문
                # tracking circle + 10m // now 20m
                if (NO_FLY_RADIUS + 5 >= haversine.haversine((self.drone_data.lat_drone, self.drone_data.lon_drone),
                                                             (self.AVOID_LAT, self.AVOID_LON))):
                    self.AVOID = True
                # 벗어나면 , 일반 추적
                else:
                    self.AVOID = False

                # 미션 좌표에 도착하면 모드를 2로 변경 ( 그전까지는 1임 )
                # 0.0000462 -> 2m
                if (self.mode == 1) and (self.plag_2 == False) and \
                        (2 >= haversine.haversine((self.drone_data.lat_drone, self.drone_data.lon_drone),
                                                  (new_gps_lat, new_gps_lon))):  # 10 은 tracking distance인데 다르게 해야할듯
                    self.mode = 2  # yaw를 회전하며 탐색 모드
                    self.plag_2 = True
                    self.plag_MISSION = False
                    time.sleep(3)
                    print("you are in mission area")

                # 2번 임무 , 사람이 detect 되지 않았으면 임의의 angle을 통해 회전
                if self.mode == 2 and self.human_detect == False:
                    new_gps_lat = self.drone_data.lat_drone
                    new_gps_lon = self.drone_data.lon_drone

                    # 금지구역 모드 X
                elif self.AVOID == False:
                    if self.human_detect == True:
                        self.mode = 3  # 추적모드

                        real_dist = math.sqrt(
                            (self.drone_data.D_x - self.H_x) ** 2 + (self.drone_data.D_y - self.H_y) ** 2)
                        error_dist = real_dist - self.following_distance

                        target_y = self.drone_data.D_y + (self.H_y - self.drone_data.D_y) * ((error_dist) / real_dist)
                        target_x = self.drone_data.D_x + (self.H_x - self.drone_data.D_x) * ((error_dist) / real_dist)

                        new_gps_lat, new_gps_lon = get_location_metres(self.drone_data.home_lat_lon, [target_x,target_y])

                        target_yaw = math.atan2((self.H_y - self.drone_data.D_y),(self.H_x - self.drone_data.D_x))# yolo를 통해 인식
                        # print("mode 3 : " , self.mode)
                    else:
                        self.mode = 5  # 사람이 추적되지않는 대기모드 일 경우 마지막 추적을 유지
                        new_gps_lat = self.drone_data.lat_drone if lat_prev == 0 else lat_prev
                        new_gps_lon = self.drone_data.lon_drone if lon_prev == 0 else lon_prev
                        # target_yaw
                        # print("mode 5 : " , self.mode)

                # 금지구역 모드
                else:
                    self.mode = 4

                    new_gps_lat = self.drone_data.lat_drone + 1  # 계산필요
                    new_gps_lon = self.drone_data.lon_drone + 1  # 계산필요

                    target_yaw = self.img_dy  # yolo를 통해 인식

                ## 1번 , 6번 수행중이라면
                # 어차피 마지막에 덮어씌우게된다.
                if self.plag_MISSION == True:
                    self.lat_person = self.MISSION_LAT
                    self.lon_person = self.MISSION_LON
                    self.drone_data.home_lat_lon = [self.drone_data.lat_drone,
                                                    self.drone_data.lon_drone]  # 365804861 # 1275257444
                    self.H_x, self.H_y = self.prev_H_x, self.prev_H_y = get_metres_location(
                        [self.drone_data.home_lat_lon[0], self.drone_data.home_lat_lon[1]],
                        [self.MISSION_LAT, self.MISSION_LON])
                    real_dist = math.sqrt(self.H_x ** 2 + self.H_y ** 2)
                    error_dist = real_dist - self.following_distance
                    target_y = self.H_y * (error_dist / real_dist)
                    target_x = self.H_x * (error_dist / real_dist)

                    new_gps_lat, new_gps_lon = get_location_metres(self.drone_data.home_lat_lon, [target_x, target_y])
                    start_target_yaw = math.atan2(self.H_y, self.H_x)
                    target_yaw = start_target_yaw
                    self.mode = 1

                if self.plag_RTH == True:
                    new_gps_lat = self.RTH_LAT
                    new_gps_lon = self.RTH_LON
                    # target_yaw = 0
                    self.mode = 6
                # if target_yaw <= 20: # 안전범위 이내라고 판단된다면
                #         target_yaw = 0 # 과도한 조절을 하지 않기 위해 설정
                # 통신을 위한 변경 코드

                # for mission 5
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

                if target_yaw < 0:
                    target_yaw +=360
                target_yaw_100 = target_yaw*100
                target_yaw_first = (target_yaw_100 >> 8) & 0xff
                target_yaw_second = target_yaw_100 & 0xff

                # self.mission_lat / self.mission_lon / roll pitch heading_angle
                # print(self.human_detect)
                # print(" calculate next gps ")
                read = str(mode_echo) + '\n' + str(self.drone_data.lat_drone) + '\n' + str(self.drone_data.lon_drone) + \
                       '\n' + str(gps_time) + '\n' + str(self.H_x) + '\n' + str(self.H_y) + '\n' + str(
                    self.drone_data.altitude)
                self.q.append(read)
                # 연산 후 바로 next_gps 전달
                self.serSTM.write(
                    [header_1, header_2, self.mode, \
                     new_lat_first, new_lat_second, new_lat_third, new_lat_fourth, \
                     new_lon_first, new_lon_second, new_lon_third, new_lon_fourth, \
                     target_yaw_first, target_yaw_second, self.lidar_distance_1, self.lidar_distance_2]
                )


class DroneData:
    def __init__(self):
        self.home_lat_lon = None
        self.lat_drone = None
        self.lon_drone = None
        self.roll = None
        self.pitch = None
        self.heading_angle = None
        self.altitude = None
        self.g_pitch = None
        self.g_roll = None
        self.D_x, self.D_y = None, None


class CoordEstimator:
    K = np.array([[980.02319542, 0, 453.44606334],
                  [0, 979.3227789, 276.15629447], [0, 0, 1]])
    R_tran = np.array([[0, 0, 1, 0], [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]])
    # remove distortion
    dist = np.array([-4.92456940e-01, 2.68723186e-01, -1.15903527e-04, -6.44152979e-04, -2.90173099e-02])
    h, w = 540, 960
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), 1, (w, h))
    x, y, w, h = roi
    # undistort
    mapx, mapy = cv2.initUndistortRectifyMap(K, dist, None, newcameramtx, (w, h), 5)
    mapx = np.array(mapx)
    mapy = np.array(mapy)
    mapx_test = np.zeros((mapx.shape[0], mapx.shape[1]), dtype=np.float32)
    mapy_test = np.zeros((mapx.shape[0], mapx.shape[1]), dtype=np.float32)
    for i in range(mapx_test.shape[0]):
        mapx_test[i, :] = [x for x in range(mapx_test.shape[1])]
    for j in range(mapy_test.shape[1]):
        mapy_test[:, j] = [y for y in range(mapy_test.shape[0])]

    mapx_ = mapx_test - mapx
    mapy_ = mapy_test - mapy
    K_inv = np.linalg.inv(newcameramtx)

    def est(self, drone_data: DroneData, foot):
        pitch, roll = - math.pi / 180 * drone_data.g_pitch, math.pi / 180 * drone_data.g_roll
        # stm front ->x, right -> y, down -> z
        yaw = math.pi / 180 * drone_data.heading_angle
        T = np.array([[drone_data.D_x], [drone_data.D_y], [-drone_data.altitude]])
        OR_G = euler_rotation_matrix_YRP(roll, pitch, yaw)
        OR_C = np.hstack((OR_G, T))
        OR_C = np.vstack((OR_C, np.array([[0, 0, 0, 1]])))
        OR_C = np.matmul(OR_C, self.R_tran)
        CR_O = np.linalg.inv(OR_C)
        A = np.dstack((CR_O[0:3, 0], CR_O[0:3, 1], CR_O[0:3, 3]))
        A = np.linalg.inv(A)
        newx = foot[0] + self.mapx_[min(int(foot[1]), self.h - 1), min(int(foot[0]), self.w - 1)]
        newy = foot[1] + self.mapy_[min(int(foot[1]), self.h - 1), min(int(foot[0]), self.w - 1)]
        xy_est = np.matmul(np.matmul(A, self.K_inv),
                           np.array([min(newx + self.x, self.w), min(newy + self.y, self.h), 1]))
        xy_est = xy_est[0]
        xy_est = xy_est / xy_est[2]
        H_x, H_y = (xy_est[0], xy_est[1])
        return [H_x, H_y]
