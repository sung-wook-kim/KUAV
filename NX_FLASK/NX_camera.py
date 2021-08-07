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

class NX(BaseCamera):
    video_source = 'test.mp4'
    
    def __init__(self):
        #self.serSTM = serial.Serial('/dev/ttyUSB0' , 115200,timeout=1)
        #self.serSTM.flush()
        self.serLIDAR = serial.Serial('/dev/ttyUSB0', 115200 , timeout =1)
        self.serLIDAR.flush()
        #self.serGIMBAL = serial.Serial('/dev/ttyUSB1', 115200 , timeout =1)
        #self.serGIMBAL.flush()
        self.distance = 0
        self.MISSION_LAT = 0
        self.MISSION_LON = 0
        self.plag_MISSION = False;
        self.RTH_LAT = 0
        self.RTH_LON = 0 
        self.plag_RTH = False;
        self.AVOID_LAT = 0
        self.AVOID_LON = 0 
        self.mode = 0  # default = 0
        self.plag_1 = False; self.plag_2 = False; self.plag_6 = False
        self.plag_9 = False
        self.AVOID = False; self.HIDE = False; self.MIDPOINT = False
        self.q = [0, 0, 0]
        # NX - GCS socket , NX = server
        self.HOST = '192.168.43.185'
        #self.HOST = '127.0.0.1'
        self.PORT = 9999
        print("Waiting")
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind(('', self.PORT))
        self.server_socket.listen()
        self.client_socket, self.addr = self.server_socket.accept()
        print("connect")
        NX.set_human_init()
        self.thread1 = threading.Thread(target=self.connectSTM)
        self.thread2 = threading.Thread(target=self.connectGCS)
        self.thread3 = threading.Thread(target=self.connectLIDAR)
        #self.thread4 = threading.Thread(target=self.connectGIMBAL)
        self.thread1.start()
        self.thread2.start()
        self.thread3.start()    
        #self.thread4.start()
        
        if os.environ.get('OPENCV_CAMERA_SOURCE'):
            NX.set_video_source(int(os.environ['OPENCV_CAMERA_SOURCE']))
        super(NX, self).__init__()
    @staticmethod
    def set_human_init():
        NX.human_detect = False
        print("human")
    @staticmethod
    def set_video_source(source):
        NX.video_source = source

    @staticmethod
    def frames():
        out, weights, imgsz, stride = \
            'inference/output', 'weights/yolov5s.pt', 640, 32
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

        # Set Dataloader
        vid_path, vid_writer = None, None
        # dataset = LoadImages(source, img_size=imgsz)
        dataset = LoadStreams(source, img_size=imgsz)
        names = model.names if hasattr(model, 'names') else model.modules.names
        colors = [[random.randint(0, 255) for _ in range(3)] for _ in range(len(names))]

        # Run inference
        t0 = time.time()
        img = torch.zeros((1, 3, imgsz, imgsz), device=device)  # init img
        _ = model(img.half() if half else img) if device.type != 'cpu' else None  # run once

        # to check the detection
        NX.human_detect = False  # if the model do detect
        crop_detect = False  # if the model do detect in crop image
        total_detect = False  # if the model do detect in original image
        w, h = int(dataset.imgs[0].shape[1]), int(dataset.imgs[0].shape[0])
        NX.img_human_center = (int(w / 2), int(h / 2))
        NX.img_human_foot = NX.img_human_center
        n = 0  # the number of detect iteration

        conf_thres = 0.4

        for path, img, im0s, vid_cap in dataset:
            img = torch.from_numpy(img).to(device)
            img = img.half() if half else img.float()  # uint8 to fp16/32
            img /= 255.0  # 0 - 255 to 0.0 - 1.0
            if img.ndimension() == 3:
                img = img.unsqueeze(0)

            # make crop version
            if not total_detect or n == 0:  # In this condition, the crop detection occur
                img0_crop = im0s.copy()
                img0_crop = img0_crop[0]
                cropsize = 50
                # image shape y,x (height and width)
                lowerbound_x = max(NX.img_human_center[0] - cropsize, 0)
                higherbound_x = min(NX.img_human_center[0] + cropsize, img0_crop.shape[1])
                lowerbound_y = max(NX.img_human_center[1] - cropsize, 0)
                higherbound_y = min(NX.img_human_center[1] + cropsize, img0_crop.shape[0])

                img0_crop = img0_crop[lowerbound_y:higherbound_y, lowerbound_x:higherbound_x]
                img0_crop = [img0_crop]
                img_crop = [letterbox(x, imgsz, stride=stride)[0] for x in img0_crop]
                img_crop = np.stack(img_crop, 0)
                img_crop = img_crop[..., ::-1].transpose((0, 3, 1, 2))  # BGR to RGB, BHWC to BCHW
                img_crop = np.ascontiguousarray(img_crop)

                # print(img.shape) # (1, 3, 480, 640)
                img_crop = torch.from_numpy(img_crop).to(device)
                img_crop = img_crop.half() if half else img_crop.float()  # uint8 to fp16/32
                img_crop /= 255.0  # 0 - 255 to 0.0 - 1.0
                if img_crop.ndimension() == 3:
                    img_crop = img_crop.unsqueeze(0)

            # Inference
            t1 = torch_utils.time_synchronized()
            pred_raw = model(img, augment=False)[0]
            if not total_detect or n == 0:  # crop condition
                pred_raw_crop = model(img_crop, augment=False)[0]

            # Apply NMS
            pred = non_max_suppression(pred_raw, conf_thres=conf_thres, iou_thres=0.5, fast=True, classes=None,
                                       agnostic=False)
            if not total_detect or n == 0:  # crop condition
                pred_crop = non_max_suppression(pred_raw_crop, conf_thres / 3, iou_thres=0.5, fast=True, classes=None,
                                                agnostic=False)

            t2 = torch_utils.time_synchronized()

            # Apply Classifier
            if classify:
                pred = apply_classifier(pred, modelc, img, im0s)

            for i, dets in enumerate(zip(pred, pred_crop)):  # detections per image
                det, det_crop = dets
                p, s, im0 = path[i], f'{i}: ', im0s[i].copy()
                if not total_detect or n == 0:  # crop condition
                    im0_crop = img0_crop[i].copy()

                s += '%gx%g ' % img.shape[2:]  # print string
                gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  #  normalization gain whwh
                if det is not None and len(det):
                    # Rescale boxes from img_size to im0 size
                    det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()
                    if not total_detect or n == 0:  # crop condition
                        det_crop[:, :4] = scale_coords(img_crop.shape[2:], det_crop[:, :4], im0_crop.shape).round()

                    # for c in det[:, -1].unique():  #probably error with torch 1.5
                    for c in det[:, -1].detach().unique():
                        n = (det[:, -1] == c).sum()  # detections per class
                        s += '%g %s, ' % (n, names[int(c)])  # add to string

                    # write result of crop version
                    human_detect = False
                    crop_detect = False
                    if not total_detect or n == 0:  # crop condition
                        for *xyxy, conf, cls in reversed(det_crop):
                            c = int(cls)  # integer class
                            label = '%s %.2f' % (names[int(cls)], conf)
                            center, foot = plot_one_box2(xyxy, im0_crop, label=label, color=colors[int(cls)],
                                                         line_thickness=1)
                            if c == 0:
                                # print(center, lowerbound_x, lowerbound_y, im0.shape, im0_crop.shape)
                                # (132, 106) 540 380 (960, 1280, 3) (200, 200, 3) -> the ratio always same, just add
                                center = (lowerbound_x + center[0], lowerbound_y + center[1])
                                foot = (lowerbound_x + foot[0], lowerbound_y + foot[1])
                                NX.img_human_center = center
                                NX.img_human_foot = foot
                                human_detect = True
                                crop_detect = True

                    # write result of original version
                    total_detect = False
                    if crop_detect is False:
                        for *xyxy, conf, cls in det:
                            label = '%s %.2f' % (names[int(cls)], conf)
                            center, foot = plot_one_box2(xyxy, im0, label=label, color=colors[int(cls)],
                                                         line_thickness=1)
                            if int(cls) == 0:
                                NX.img_human_center = center
                                NX.img_human_foot = foot
                                human_detect = True
                                total_detect = True

                    fontFace = cv2.FONT_HERSHEY_SIMPLEX
                    fontScale = 0.5
                    thickness = 2
                    textSize, baseline = cv2.getTextSize("FPS", fontFace, fontScale, thickness)

                    if crop_detect:
                        im0[lowerbound_y:higherbound_y, lowerbound_x:higherbound_x] = im0_crop
                        cv2.putText(im0, 'Crop', (lowerbound_x, lowerbound_y + textSize[1]), fontFace,
                                    fontScale,
                                    (255, 0, 255), thickness)
                        # print((human_center[0], im0.shape[0] / 2), (im0.shape[1] / 2, im0.shape[0] / 2))

                    img_dx = NX.img_human_foot[0] - w / 2
                    img_dy = NX.img_human_foot[1] - h / 2
                    cv2.line(im0,
                             (int(img_dx + im0.shape[1] / 2), int(im0.shape[0] / 2)),
                             (int(im0.shape[1] / 2), int(im0.shape[0] / 2)), (0, 0, 255),
                             thickness=1, lineType=cv2.LINE_AA)
                    cv2.line(im0,
                             (int(im0.shape[1] / 2), int(img_dy + im0.shape[0] / 2)),
                             (int(im0.shape[1] / 2), int(im0.shape[0] / 2)), (0, 0, 255),
                             thickness=1, lineType=cv2.LINE_AA)
                    cv2.putText(im0, 'Human Detection ' + str(human_detect), (10, 10 + textSize[1]), fontFace,
                                fontScale,
                                (255, 0, 255), thickness)
                    cv2.putText(im0, 'Crop Detection ' + str(crop_detect), (10, 25 + textSize[1]), fontFace, fontScale,
                                (255, 0, 255), thickness)
                    cv2.putText(im0, ' Total Detection ' + str(total_detect), (10, 40 + textSize[1]), fontFace,
                                fontScale,
                                (255, 0, 255), thickness)

                print('%sDone. (%.3fs)' % (s, t2 - t1), NX.img_human_center)

            yield cv2.imencode('.jpg', im0)[1].tobytes()

    def connectGCS(self):
        time.sleep(15)
        while True:
            try:
                sendingMsg = self.q.pop(-1)  # sendingmsg = 'mode \n lat_drone \n lon_drone \n gps_time \n lat_person \n lon_person \n altitude'
                gcs = self.client_socket.recv(1024).decode().split('\n')
                print(gcs)
                NX.human_detect = True
                plag = gcs[0]
                if plag == '1' and self.plag_1 == False:
                    self.mode = 1  # 임무 장소 이동 , 30M 고도 유지
                    self.plag_1 = True
                    self.MISSION_LAT = plag[1]
                    self.MISSION_LON = plag[2]
                    self.plag_MISSION = True
                elif plag == '6' and self.plag_6 == False:
                    self.mode = 6  # RTH = 착륙
                    self.plag_6 = True
                    self.RTH_LAT = plag[1]
                    self.RTH_LAT = plag[2]
                    self.plag_RTH = True
                elif plag == '9' and self.plag_9 == False:
                    self.mode = 9  # 비상 모터 정지
                    self.plag_9 = True
                print(self.mode)  # 현재 모드 확인용
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
                    self.distance = np.int16(recv[2] + np.int16(recv[3] << 8))
                    print(f'distance = {self.distance}cm')
                    time.sleep(0.2)
                    self.serLIDAR.reset_input_buffer()
    def connectSTM(self):
        # 연산 속도는 과연,,.?!
        header_1 = 0x44
        header_2 = 0x77
        while True:
            # countSTM = self.serSTM.in_waiting
            # if countSTM > 18:
            #     recvSTM = self.serSTM.read(19)
            #     self.serSTM.reset_input_buffer() 
            #     if recvSTM[0] == 0x88 and recvSTM[1] == 0x18:
            #         mode_echo = np.int16(recvSTM[2])

            #         lat_drone = np.int16(np.int16(recvSTM[3] << 24) + np.int16(recvSTM[4] << 16) + np.int16(recvSTM[5] << 8 ) + recvSTM[6])
            #         lon_drone = np.int16(np.int16(recvSTM[7] << 24) + np.int16(recvSTM[8] << 16) + np.int16(recvSTM[9] << 8 ) + recvSTM[10])
            #         gps_time = np.int16(np.int16(recvSTM[11] << 24) + np.int16(recvSTM[12] << 16) + np.int16(recvSTM[13] << 8 ) + recvSTM[14])

            #         roll = np.int16(recvSTM[15])
            #         pitch = np.int16(recvSTM[16])
            #         heading_angle = np.int16(recvSTM[17])
            #         altitude = np.int16(recvSTM[18])
 
            #         print(mode_echo,lat_drone,lon_drone,gps_time,roll,pitch,heading_angle,altitude)
            #         self.serSTM.reset_input_buffer()
            lat_drone = 38.00123112 ; lon_drone = 127.12312342 ; 
            lat_person = 32.123123213 ; lon_person = 123.2323123
            gps_time = 32112342 
            roll = 321 ; pitch = 122 ; heading_angle =123 ; altitude =21
            # 특정 범위에 드론이 들어가면 , AVOID 모드 AVOID on ,off 이므로 확실히 구분 된 조건문
            if lat_drone == self.AVOID_LAT and lon_drone == self.AVOID_LON:
                self.AVOID = True
            # 벗어나면 , 일반 추적
            elif lat_drone != self.AVOID_LAT and lon_drone != self.AVOID_LON:
                self.AVOID = False 


            # 과도한 연산 방지를 위해 설정 ok
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
                    
                    yaw_error = 500000000 # yolo를 통해 인식
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
                
                lat_person = 500000 # 계산필요
                lon_person = 500000 # 계산필요
     
                yaw_error = 500000000 # yolo를 통해 인식
            
            new_gps_lat = 37.123124512
            new_gps_lon = 127.1232131
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
            # new_lat_first = (new_gps_lat >> 24) & 0xff ; new_lat_second = (new_gps_lat >> 16) & 0xff
            # new_lat_third = (new_gps_lat >> 8) & 0xff  ; new_lat_fourth = new_gps_lat & 0xff

            # new_lon_first = (new_gps_lon >> 24) & 0xff ; new_lon_second = (new_gps_lon >> 16) & 0xff
            # new_lon_third = (new_gps_lon >> 8) & 0xff  ; new_lon_fourth = new_gps_lon & 0xff
            
            read = str(self.mode) + '\n' + str(lat_drone) + '\n' + str(lon_drone) + '\n' + str(gps_time) +'\n' +  str(lat_person) + '\n' + str(
                lon_person) + '\n' + str(altitude)
            print(NX.human_detect)
            self.q.append(read)
            time.sleep(0.2)
            print("STM")
            # 연산 후 바로 next_gps 전달
            # self.ser.write(
            #     [header_1,header_2,self.mode,\
            #     new_lat_first,new_lat_second,new_lat_third,new_lat_fourth,\
            #     new_lon_first,new_lon_second,new_lon_third,new_lon_fourth,\
            #     yaw_error , self.distance]
            # )
