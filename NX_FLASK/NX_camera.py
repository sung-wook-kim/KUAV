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
from utils.datasets import *
from utils.utils import *
from gimbalcmd import *

class NX(BaseCamera):
    video_source = 'test.mp4'
    
    def __init__(self):
        self.serSTM = serial.Serial('/dev/ttyUSB0' , 115200,timeout=1)
        self.serSTM.flush()
        print("stm")
        self.serLIDAR = serial.Serial('/dev/ttyUSB1', 115200 , timeout =1)
        self.serLIDAR.flush()
        print("lidar")
        ##gimbalstart
        sys.stdout.flush()
        ser = serialinit()
        safemode = responsetest(ser)
        paramstore(safemode)
        sleepmultipliercalc()
        intervalcalc()
        setpitchrollyaw(0,0,0)
        print("gimbal init")
        self.lidar_distance_1 = 0 ; self.lidar_distance_2 = 0
        self.MISSION_LAT = 0 ; self.MISSION_LON = 0
        self.plag_MISSION = False; self.plag_RTH = False
        self.RTH_LAT = 2353 ; self.RTH_LON = 235
        self.AVOID_LAT = 1234 ; self.AVOID_LON = 1234 
        self.mode = 0  # default = 0
        self.plag_1 = False; self.plag_2 = False; self.plag_6 = False
        self.plag_9 = False
        self.AVOID = False
        self.q = [0, 0, 0]
        # NX - GCS socket , NX = server
        self.HOST = '223.171.80.232'
        self.PORT = 9998
        print("Waiting")
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind(('', self.PORT))
        self.server_socket.listen()
        self.client_socket, self.addr = self.server_socket.accept()
        print("connect")
        NX.set_human_init()
        self.threadSTM = threading.Thread(target=self.connectSTM)
        self.threadGCS = threading.Thread(target=self.connectGCS)
        self.threadLIDAR = threading.Thread(target=self.connectLIDAR)
        self.threadGIMBAL = threading.Thread(target=self.connectGIMBAL)
        self.threadSTM.start()
        self.threadGCS.start()
        self.threadLIDAR.start()
        self.threadGIMBAL.start()
        if os.environ.get('OPENCV_CAMERA_SOURCE'):
            NX.set_video_source(int(os.environ['OPENCV_CAMERA_SOURCE']))
        super(NX, self).__init__()

    @staticmethod
    def set_human_init():
        NX.human_detect = False
        NX.img_dx = 0
        NX.img_dy = 0
        NX.gimbal_plag = False
        print("human")

    @staticmethod
    def set_video_source(source):
        NX.video_source = source

    @staticmethod
    def frames():
        out, weights, imgsz, stride = \
            'inference/output', '/home/drone/kuav/NX_FLASK/weights/yolov5s.pt', 640, 32
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

        # Second-stage classifier
        classify = False
        if classify:
            modelc = torch_utils.load_classifier(name='resnet101', n=2)  # initialize
            modelc.load_state_dict(torch.load('weights/resnet101.pt', map_location=device)['model'])  # load weights
            modelc.to(device).eval()

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
        NX.human_detect = False  # if the model do detect
        w, h = int(dataset.imgs[0].shape[1]), int(dataset.imgs[0].shape[0])
        NX.img_human_center = (int(w / 2), int(h / 2))
        NX.img_human_foot = NX.img_human_center
        NX.img_dx = 0
        NX.img_dy = 0
        n = 0  # the number of detect iteration
        conf_thres = 0.4
        fontFace = cv2.FONT_HERSHEY_SIMPLEX
        fontScale = 0.5
        thickness = 2
        textSize, baseline = cv2.getTextSize("FPS", fontFace, fontScale, thickness)

        for path, img, im0s, vid_cap in dataset:
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

            # Apply Classifier
            if classify:
                pred = apply_classifier(pred, modelc, img, im0s)

            for i, det in enumerate(pred):  # detections per image
                p, s, im0 = path[i], f'{i}: ', im0s[i].copy()

                s += '%gx%g ' % img.shape[2:]  # print string
                gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  #  normalization gain whwh
                if det is not None and len(det):
                    # Rescale boxes from img_size to im0 size
                    det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

                    # for c in det[:, -1].unique():  #probably error with torch 1.5
                    for c in det[:, -1].detach().unique():
                        n = (det[:, -1] == c).sum()  # detections per class
                        s += '%g %s, ' % (n, names[int(c)])  # add to string

                    # write result of crop version
                    NX.human_detect = False
                    # write result of original version
                    for *xyxy, conf, cls in det:
                        label = '%s %.2f' % (names[int(cls)], conf)
                        center, foot = plot_one_box2(xyxy, im0, label=label, color=colors[int(cls)],
                                                     line_thickness=1)
                        if int(cls) == 0:
                            NX.img_human_center = center
                            NX.img_human_foot = foot
                            NX.human_detect = True
                            NX.img_dx = NX.img_human_center[0] - w / 2
                            NX.img_dy = NX.img_human_center[1] - h / 2
                            NX.gimbal_plag = True
                    cv2.line(im0,
                             (int(NX.img_dx + im0.shape[1] / 2), int(im0.shape[0] / 2)),
                             (int(im0.shape[1] / 2), int(im0.shape[0] / 2)), (0, 0, 255),
                             thickness=1, lineType=cv2.LINE_AA)
                    cv2.line(im0,
                             (int(im0.shape[1] / 2), int(NX.img_dy + im0.shape[0] / 2)),
                             (int(im0.shape[1] / 2), int(im0.shape[0] / 2)), (0, 0, 255),
                             thickness=1, lineType=cv2.LINE_AA)
                    cv2.putText(im0, 'Human Detection ' + str(NX.human_detect), (10, 10 + textSize[1]), fontFace,
                                fontScale,
                                (255, 0, 255), thickness)

                print('%sDone. (%.3fs)' % (s, t2 - t1), NX.img_human_center)

            yield cv2.imencode('.jpg', im0)[1].tobytes()

    def connectGIMBAL(self):
        target_pitch = 0
        target_yaw = 0
        dy_prev = 0
        dx_prev  = 0
        while True:
            if NX.gimbal_plag == True:
                NX.gimbal_plag = False
                pitch_gain = 0.01
                yaw_gain = -0.02
#                pitch_d = -0.0025 
#                yaw_d = 0.005
#                dy_term = NX.img_dy - dy_prev
#                dx_term = NX.img_dx - dx_prev
                target_pitch += (pitch_gain * NX.img_dy )#+ dy_term * pitch_d) 
                target_yaw += (yaw_gain * NX.img_dx )#+ dx_term * yaw_d)
                #   Pitch     Roll      Yaw      axislimit
                # [Min, Max, Min, Max, Min, Max]
                target_pitch = min(max(axislimit[0], target_pitch), axislimit[1])
                target_yaw = min(max(axislimit[4], target_yaw), axislimit[5])
                print(f"img_dx {NX.img_dx}, img_dy {NX.img_dy}, target_pitch {target_pitch}, target_yaw {target_yaw}")
                setpitchrollyaw(target_pitch, 0, target_yaw)
#                dy_prev = NX.img_dy
#                dx_prev = NX.img_dx
            else:
                time.sleep(0.05)

    def connectGCS(self):
        time.sleep(10)
        while True:
            try:
                sendingMsg = self.q.pop(-1)  # sendingmsg = 'mode \n lat_drone \n lon_drone \n gps_time \n lat_person \n lon_person \n altitude \n detection'
                gcs = self.client_socket.recv(1024).decode().split('\n')
                print(gcs)
                plag = gcs[0]
                if plag == '1' and self.plag_1 == False:
                    self.mode = 1  # 임무 장소 이동 , 30M 고도 유지
                    self.plag_1 = True
                    self.MISSION_LAT = int(gcs[1])
                    self.MISSION_LON = int(gcs[2])
                    self.plag_MISSION = True
                elif plag == '6' and self.plag_6 == False:
                    self.mode = 6  # RTH = 착륙
                    self.plag_6 = True
                    self.RTH_LAT = int(gcs[1])
                    self.RTH_LAT = int(gcs[2])
                    self.plag_RTH = True
                elif plag == '9' and self.plag_9 == False:
                    self.mode = 9  # 비상 모터 정지
                    self.plag_9 = True
                print("mode : " ,self.mode)  # 현재 모드 확인용
                self.client_socket.sendall(sendingMsg.encode())
            except Exception as e:
                # GCS connection off , connection waiting
                print("connection error")
                self.client_socket.close()
                self.client_socket, self.addr = self.server_socket.accept()

    def connectLIDAR(self):
        while True:
            count = self.serLIDAR.in_waiting 
            if count > 8: # 버퍼에 9바이트 이상 쌓이면 
                recv = self.serLIDAR.read(9) # read
                self.serLIDAR.reset_input_buffer() # 리셋
                if recv[0] == 0x59 and recv[1] == 0x59:  # python3
                    self.lidar_distance_1 = recv[2] # np.int16(recv[2] + np.int16(recv[3] << 8))
                    self.lidar_distance_2 = recv[3]
                    time.sleep(0.2)
                    print("LIDAR  : ",self.lidar_distance_1)
                    self.serLIDAR.reset_input_buffer()

    def connectSTM(self):
        header_1 = 0x88
        header_2 = 0x18
        lat_drone = 1 ; lon_drone =1 ; gps_time = 1 ; roll = 1 ; pitch = 1 ; heading_angle = 1 ; altitude = 1 ; voltage = 1
        while True:
            countSTM = self.serSTM.in_waiting
            if countSTM > 34:
                recvSTM = self.serSTM.read(35)
                self.serSTM.reset_input_buffer() 
                if recvSTM[0] == 0x88 and recvSTM[1] == 0x18:
                    mode_echo = np.int16(recvSTM[2])

                    lat_drone = np.int32(np.int32(recvSTM[3] << 24) + np.int32(recvSTM[4] << 16) + np.int32(recvSTM[5] << 8 ) + recvSTM[6]) / 10**7
                    lon_drone = np.int32(np.int32(recvSTM[7] << 24) + np.int32(recvSTM[8] << 16) + np.int32(recvSTM[9] << 8 ) + recvSTM[10]) / 10**7
                    gps_time = np.int32(np.int32(recvSTM[11] << 24) + np.int32(recvSTM[12] << 16) + np.int32(recvSTM[13] << 8 ) + recvSTM[14])

                    roll = np.int32(np.int32(recvSTM[15] << 24) + np.int32(recvSTM[16] << 16) + np.int32(recvSTM[17] << 8 ) + recvSTM[18])
                    pitch = np.int32(np.int32(recvSTM[19] << 24) + np.int32(recvSTM[20] << 16) + np.int32(recvSTM[21] << 8 ) + recvSTM[22])
                    heading_angle = np.int32(np.int32(recvSTM[23] << 24) + np.int32(recvSTM[24] << 16) + np.int32(recvSTM[25] << 8 ) + recvSTM[26])
                    altitude = np.int32(np.int32(recvSTM[27] << 24) + np.int32(recvSTM[28] << 16) + np.int32(recvSTM[29] << 8 ) + recvSTM[30])
                    voltage = np.int32(np.int32(recvSTM[31] << 24) + np.int32(recvSTM[32] << 16) + np.int32(recvSTM[33] << 8 ) + recvSTM[34])
                    print("From STM : " , mode_echo , lat_drone , lon_drone , gps_time , roll , pitch , heading_angle , altitude , voltage)
                    read = str(mode_echo) + '\n' + str(lat_drone) + '\n' + str(lon_drone) + '\n' + str(gps_time) +'\n' +  str(lat_drone+1) + '\n' + str(
                        lon_drone+1) + '\n' + str(altitude)
                    self.serSTM.reset_input_buffer()
                # 특정 범위에 드론이 들어가면 , AVOID 모드 AVOID on ,off 이므로 확실히 구분 된 조건문
                if lat_drone == self.AVOID_LAT or lon_drone == self.AVOID_LON:
                    self.AVOID = True
                # 벗어나면 , 일반 추적
                else:
                    self.AVOID = False 

                # 미션 좌표에 도착하면 모드를 2로 변경 ( 그전까지는 1임 )
                if self.plag_2 == False and  lat_drone == self.MISSION_LAT and lon_drone == self.MISSION_LON:
                    self.mode = 2  # yaw를 회전하며 탐색 모드
                    self.plag_2 = True
                    self.plag_MISSION = False

                # 2번 임무 , 사람이 detect 되지 않았으면 임의의 값을 통해 회전 
                if self.mode == 2 and NX.human_detect == False:
                    new_gps_lat = lat_drone
                    new_gps_lon = lon_drone
                    yaw_error = 20 

                # 금지구역 모드 X 
                if self.AVOID == False:
                    if NX.human_detect == True:
                        self.mode = 3 # 추적모드
                        new_gps_lat = lat_drone + 1 # 계산필요
                        new_gps_lon = lon_drone + 1 # 계산필요
                        
                        lat_person = 500000 # 계산필요
                        lon_person = 500000 # 계산필요
                        
                        yaw_error = NX.img_dy  # yolo를 통해 인식
                    else: 
                        self.mode = 5 # 대기모드
                        new_gps_lat = lat_drone
                        new_gps_lon = lat_drone
                        yaw_error = 0

                # 금지구역 모드
                else:
                    self.mode = 4
                    new_gps_lat = lat_drone + 1 # 계산필요
                    new_gps_lon = lon_drone + 1 # 계산필요
                    
                    lat_person = 500000 # 계산필요            # time.sleep(0.2)
                    lon_person = 500000 # 계산필요
        
                    yaw_error = NX.img_dy # yolo를 통해 인식
                
                new_gps_lat = 371231245
                new_gps_lon = 1271232131
                ## 1번 , 6번 수행중이라면
                if self.plag_MISSION == True:
                    new_gps_lat = self.MISSION_LAT
                    new_gps_lon = self.MISSION_LON
                    yaw_error = 0
                    self.mode = 1

                if self.plag_RTH == True:
                    new_gps_lat = self.RTH_LAT
                    new_gps_lon = self.RTH_LON 
                    yaw_error = 0
                    self.mode = 6
                # if yaw_error <= 20: # 안전범위 이내라고 판단된다면
                #         yaw_error = 0 # 과도한 조절을 하지 않기 위해 설정
                
                # 통신을 위한 변경 코드
                new_lat_first = (new_gps_lat >> 24) & 0xff ; new_lat_second = (new_gps_lat >> 16) & 0xff
                new_lat_third = (new_gps_lat >> 8) & 0xff  ; new_lat_fourth = new_gps_lat & 0xff

                new_lon_first = (new_gps_lon >> 24) & 0xff ; new_lon_second = (new_gps_lon >> 16) & 0xff
                new_lon_third = (new_gps_lon >> 8) & 0xff  ; new_lon_fourth = new_gps_lon & 0xff
               
                lat_person = lat_drone + 1  
                lon_person = lon_drone + 1         
                read = str(self.mode) + '\n' + str(lat_drone) + '\n' + str(lon_drone) + '\n' + str(gps_time) +'\n' +  str(lat_person) + '\n' + str(
                    lon_person) + '\n' + str(altitude)
                print(NX.human_detect)
                self.q.append(read)
                # 연산 후 바로 next_gps 전달
                self.serSTM.write(
                    [header_1,header_2,self.mode,\
                    new_lat_first,new_lat_second,new_lat_third,new_lat_fourth,\
                    new_lon_first,new_lon_second,new_lon_third,new_lon_fourth,\
                    yaw_error , self.lidar_distance_1 , self.lidar_distance_2]
                )
