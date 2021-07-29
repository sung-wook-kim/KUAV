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
        #self.ser = serial.Serial('/dev/ttyUSB0' , 115200,timeout=1)
        #self.ser.flush()
        self.MISSION_LAT = 0
        self.MISSION_LON = 0
        self.plag_MISSION = False;
        self.RTH_LAT = 0
        self.RTH_LON = 0
        self.plag_RTH = False;
        self.mode = 0  # default = 0
        self.plag_1 = False; self.plag_2 = False; self.plag_3 = False
        self.plag_4 = False; self.plag_5 = False; self.plag_6 = False
        self.plag_9 = False
        self.AVOID = False; self.HIDE = False; self.MIDPOINT = False
        self.q = [0, 0, 0]
        # NX - GCS socket , NX = server
        #self.HOST = '192.168.43.185'
        self.HOST = '127.0.0.1'
        self.PORT = 9999
        print("Waiting")
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind(('', self.PORT))
        self.server_socket.listen()
        self.client_socket, self.addr = self.server_socket.accept()
        print("connect")
        self.thread1 = threading.Thread(target=self.connectSTM)
        self.thread2 = threading.Thread(target=self.connectGCS)
        self.thread1.start()
        self.thread2.start()
        if os.environ.get('OPENCV_CAMERA_SOURCE'):
            NX.set_video_source(int(os.environ['OPENCV_CAMERA_SOURCE']))
        super(NX, self).__init__()
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
        human_detect = False  # if the model do detect
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
        while True:
            try:
                sendingMsg = self.q.pop(-1)  # sendingmsg = 'mode \n lat_drone \n lon_drone \n GPS_time \n lat_person \n lon_person \n altitude'
                gcs = self.client_socket.recv(1024).decode().split('\n')
                print(gcs)
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

    def connectSTM(self):
        i = 0
        header_1 = 0x44
        header_2 = 0x77
        while True:
            i += 1
            sync1 = int(self.ser.read(1).hex(),16)
            if sync1 == 0x88:
                sync2 = int(self.ser.read(1).hex(),16)
                if sync2 == 0x18:
                    mode_echo = int(self.ser.read(1).hex(),16) & 0xff

                    lat_1 = int(self.ser.read(1).hex(),16) & 0xff
                    lat_2 = int(self.ser.read(1).hex(),16) & 0xff 
                    lat_3 = int(self.ser.read(1).hex(),16) & 0xff
                    lat_4 = int(self.ser.read(1).hex(),16) & 0xff

                    lon_1 = int(self.ser.read(1).hex(),16) & 0xff
                    lon_2 = int(self.ser.read(1).hex(),16) & 0xff
                    lon_3 = int(self.ser.read(1).hex(),16) & 0xff
                    lon_4 = int(self.ser.read(1).hex(),16) & 0xff
            
                    time_1 = int(self.ser.read(1).hex(),16) & 0xff
                    time_2 = int(self.ser.read(1).hex(),16) & 0xff
                    time_3 = int(self.ser.read(1).hex(),16) & 0xff
                    time_4 = int(self.ser.read(1).hex(),16) & 0xff

                    roll = int(self.ser.read(1).hex(),16) & 0xff
                    pitch = int(self.ser.read(1).hex(),16) & 0xff
                    heading_angle = int(self.ser.read(1).hex(),16) & 0xff
                    altitude = int(self.ser.read(1).hex(),16) & 0xff

                    lat_drone = lat_1 << 24 | lat_2 << 16 | lat_3 << 8 | lat_4
                    lon_drone = lon_1 << 24 | lon_2 << 16 | lon_3 << 8 | lon_4
                    GPS_time = time_1 << 24 | time_2 << 16 | time_3 << 8 | time_4

                    print(mode_echo,lat_drone,lon_drone,GPS_time,roll,pitch,heading_angle,altitude)
            # temp = [1, 370001000,  38000000, 82134223414, 120, 130, 150, 200]
            # mode_echo = temp[0] ; lat_drone = temp[1]; lon_drone = temp[2]; 
            # GPS_time = temp[3]
            # roll = temp[4]; pitch = temp[5]; heading_angle = temp[6]; altitude = temp[7]
            # if time.time() - start >= 3 and plag_2 == False:
            #    mode = 2 # yaw를 회전하며 탐색 모드
            #    plag_2 = True

            if self.plag_2 == False and  lat_drone == self.MISSION_LAT and lon_drone == self.MISSION_LON:
                self.mode = 2  # yaw를 회전하며 탐색 모드
                self.plag_2 = True
                self.plag_MISSION = False

            elif self.plag_3 == False and self.MIDPOINT == True and self.AVOID == False:
                self.mode = 3  # 추적 모드
                self.plag_3 = True

            elif self.plag_4 == False and self.AVOID == True:
                self.mode = 4  # 금지 구역 모드
                self.plag_4 = True
                self.plag_3 = False
            elif self.plag_5 == False and self.HIDE == True:
                self.mode = 5  # 임무 대기 모드 ( 파라솔 )
                self.plag_5 = True
                self.plag_3 = False  # 추적 모드 재 가동을 위한 플래그 off

            # 2번 임무의 경우 2번이 가장문제네
            if self.mode ==2:
                pass
            
            # 금지구역 모드 X 
            if self.AVOID == False: 
                new_gps_lat = lat_drone + 1 # 계산필요
                new_gps_lon = lon_drone + 1 # 계산필요
                
                lat_person = 500000 # 계산필요
                lon_person = 500000 # 계산필요
                
                yaw_error = 500000000 # yolo를 통해 인식

                if yaw_error <= 20: # 안전범위 이내라고 판단된다면
                    yaw_error = 0 # 과도한 조절을 하지 않기 위해 설정

            # 금지구역 모드
            else:
                new_gps_lat = lat_drone + 1 # 계산필요
                new_gps_lon = lon_drone + 1 # 계산필요
                
                lat_person = 500000 # 계산필요
                lon_person = 500000 # 계산필요
     
                yaw_error = 500000000 # yolo를 통해 인식

                if yaw_error <= 20: # 안전범위 이내라고 판단된다면
                    yaw_error = 0 # 과도한 조절을 하지 않기 위해 설정
                
                
            ## 1번 , 6번 수행중이라면
            if self.plag_MISSION == True:
                new_gps_lat = self.MISSION_LAT
                new_gps_lon = self.MISSION_LON
                yaw_error = 0

            if self.plag_RTH == True:
                new_gps_lat = self.RTH_LAT
                new_gps_lon = self.RTH_LON 
                yaw_error = 0
            
            new_lat_first = (new_gps_lat >> 24) & 0xff;
            new_lat_second = (new_gps_lat >> 16) & 0xff
            new_lat_third = (new_gps_lat >> 8) & 0xff;
            new_lat_fourth = new_gps_lat & 0xff

            new_lon_first = (new_gps_lon >> 24) & 0xff;
            new_lon_second = (new_gps_lon >> 16) & 0xff
            new_lon_third = (new_gps_lon >> 8) & 0xff;
            new_lon_fourth = new_gps_lon & 0xff
            
            read = str(self.mode) + '\n' + str(lat_drone) + '\n' + str(lon_drone) + '\n' + str(GPS_time) +'\n' +  str(lat_person) + '\n' + str(
                lon_person) + '\n' + str(altitude)
            self.q.append(read)

            ## 연산 후 바로 next_gps 전달
            # ser.write(
            #     [header_1,header_2,self.mode,\
            #     new_lat_first,new at_second,new_lat_third,new_lat_fourth,\
            #     new_lon_first,new_lon_second,new_lon_third,new_lon_fourth,\
            #     yaw_error]
            # )
        
            # df = pd.DataFrame()
            # df['gps_lat'] = gps_lat
            # df['gps_lon'] = gps_lon
            # df['alt'] = alt
            # df.to_csv("GPSdata.csv")
