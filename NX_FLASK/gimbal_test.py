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
        ##gimbalstart
        sys.stdout.flush()
        ser = serialinit()
        safemode = responsetest(ser)
        paramstore(safemode)
        sleepmultipliercalc()
        intervalcalc()
        setpitchrollyaw(0,0,0)
        print("gimbal init")
        # NX - GCS socket , NX = server
        #self.HOST = '223.171.80.232'
        self.HOST = '192.168.0.196'
        self.PORT = 9998
        NX.set_human_init()
        self.threadGIMBAL = threading.Thread(target=self.connectGIMBAL)
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
                    # NX.human_detect = False
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
                pitch_gain = 0.007
                yaw_gain = -0.007
#                pitch_d = -0.0025 
#                yaw_d = 0.005
#                dy_term = NX.img_dy - dy_prev
#                dx_term = NX.img_dx - dx_prev
                target_pitch += round((pitch_gain * NX.img_dy ),2)#+ dy_term * pitch_d) 
                target_yaw += round((yaw_gain * NX.img_dx ),2) #+ dx_term * yaw_d)
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
    # 10hz time.sleep(0.1)