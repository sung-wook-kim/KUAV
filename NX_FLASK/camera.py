import os
import cv2
from base_camera import BaseCamera
import torch
import torch.nn as nn
import torchvision
import numpy as np
import argparse
from utils.datasets import *
from utils.utils import *


class Camera(BaseCamera):
    video_source = 'test.mp4'
    def __init__(self):
        super(Camera, self).__init__()

    def frames(self):
        out, weights, imgsz = \
            'inference/output', 'weights/yolov5s.pt', 640
        # source = 'test.mp4'
        source = 0
        device = torch_utils.select_device()

        stride = 32

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
        img_human_center = (int(w / 2), int(h / 2))
        img_human_foot = img_human_center
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
                lowerbound_x = max(img_human_center[0] - cropsize, 0)
                higherbound_x = min(img_human_center[0] + cropsize, img0_crop.shape[1])
                lowerbound_y = max(img_human_center[1] - cropsize, 0)
                higherbound_y = min(img_human_center[1] + cropsize, img0_crop.shape[0])

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

                # save_path = str(Path(out) / Path(p).name)
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
                                img_human_center = center
                                img_human_foot = foot
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
                                img_human_center = center
                                img_human_foot = foot
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

                    img_dx = img_human_foot[0] - w / 2
                    img_dy = img_human_foot[1] - h / 2
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

                print('%sDone. (%.3fs)' % (s, t2 - t1))

            yield cv2.imencode('.jpg', im0)[1].tobytes()
