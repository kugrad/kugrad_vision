#! /usr/bin/env python

import rospy
import pathlib 
from geometry_msgs.msg import Pose2D

import argparse
import time
from pathlib import Path

import cv2
import torch
import torch.backends.cudnn as cudnn
from numpy import random

from models.experimental import attempt_load
from utils.datasets import LoadStreams, LoadImages
from utils.general import check_img_size, check_requirements, check_imshow, non_max_suppression, apply_classifier, \
    scale_coords, xyxy2xywh, strip_optimizer, set_logging, increment_path
from utils.plots import plot_one_box
from utils.torch_utils import select_device, load_classifier, time_synchronized, TracedModel

import matplotlib
# matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

import time
from copy import copy
import os.path

import numpy as np

coord_pub = None

# box의 x, y 좌표 저장하는 클래스
class position:
    x=0
    y=0
    def __init__(self,x,y):
        self.x=x
        self.y=y
    def getX(self):
        return self.x
    def getY(self):
        return self.y
    
# RGB 히스토그램과 x좌표 저장하는 클래스
class histogram_class:
    histogram=[]
    center_x=0
    def __init__(self,histogram,center_x):
        self.histogram=histogram
        self.center_x=center_x


def detect(save_img=False):
    source, weights, view_img, save_txt, imgsz, trace = opt.source, opt.weights, opt.view_img, opt.save_txt, opt.img_size, not opt.no_trace
    save_img = not opt.nosave and not source.endswith('.txt')  # save inference images
    webcam = source.isnumeric() or source.endswith('.txt') or source.lower().startswith(
        ('rtsp://', 'rtmp://', 'http://', 'https://'))

    # Directories
    save_dir = Path(increment_path(Path(opt.project) / opt.name, exist_ok=opt.exist_ok))  # increment run
    (save_dir / 'labels' if save_txt else save_dir).mkdir(parents=True, exist_ok=True)  # make dir

    # Initialize
    set_logging()
    device = select_device(opt.device)
    half = device.type != 'cpu'  # half precision only supported on CUDA

    # Load model
    model = attempt_load(weights, map_location=device)  # load FP32 model
    stride = int(model.stride.max())  # model stride
    imgsz = check_img_size(imgsz, s=stride)  # check img_size

    if trace:
        model = TracedModel(model, device, opt.img_size)

    if half:
        model.half()  # to FP16

    # Second-stage classifier
    classify = False
    if classify:
        modelc = load_classifier(name='resnet101', n=2)  # initialize
        modelc.load_state_dict(torch.load('weights/resnet101.pt', map_location=device)['model']).to(device).eval()
    
    # Set Dataloader
    vid_path, vid_writer = None, None
    if webcam:
        view_img = check_imshow()
        cudnn.benchmark = True  # set True to speed up constant image size inference
        dataset = LoadStreams(source, img_size=imgsz, stride=stride)
    else:
        dataset = LoadImages(source, img_size=imgsz, stride=stride)
    
    # Get names and colors
    names = model.module.names if hasattr(model, 'module') else model.names
    colors = [[random.randint(0, 255) for _ in range(3)] for _ in names]
    
    # print(names)

    # Run inference
    if device.type != 'cpu':
        model(torch.zeros(1, 3, imgsz, imgsz).to(device).type_as(next(model.parameters())))  # run once
    old_img_w = old_img_h = imgsz
    old_img_b = 1
    
    # 여기에 있는 배열의 각 저장된 하나씩은 frame의 정보다. 
    # 동영상 용 frame 별 각 box 당 히스토그램 저장 위한 list
    redhist=[]
    bluehist=[]
    greenhist=[]
    
    # 전체 detecting 되는 히스토그램 개수
    cnt=0   
    
    # frame당 박스 중앙 좌표 저장한 리스트 저장하는 리스트
    centercoord=[]
    
    # frame당 박스 이미지 저장하는 리스트 저장하는 리스트
    frameboximage=[]
    
    # 히스토그램 비교 로직에서 현재 몇 번째 frame인지 확인하기 위한 변수
    framenum=0
    
    #몇번째 frame 전과 비교할 것인지 정하는 상수
    beforeframe=15
    
    # 변화 검출 비율
    detectrate=0.4
    
    # 박스 같은 박스인지 확인 시 좌표 차이
    coordrate=20
    
    # (test 기능 구현) change detct시 박스 색 바꾸기 
    # xyxy, im0 저장 위한 리스트
    change_color_list=[]
    
    # box 색 유지 위한 변수
    color_cnt=0
    
    #change detect된 횟수
    detect_cnt=0
    
    #box 이미지 저장시 이름 변경 위한 변수
    image_num=0

    t0 = time.time()

    # frame들 loop 돌리기
    for path, img, im0s, vid_cap in dataset:
        
        img = torch.from_numpy(img).to(device)
        img = img.half() if half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        # Warmup
        if device.type != 'cpu' and (old_img_b != img.shape[0] or old_img_h != img.shape[2] or old_img_w != img.shape[3]):
            old_img_b = img.shape[0]
            old_img_h = img.shape[2]
            old_img_w = img.shape[3]
            for i in range(3):
                model(img, augment=opt.augment)[0]
        
        # Inference
        t1 = time_synchronized()
        with torch.no_grad():   # Calculating gradients would cause a GPU memory leak
            pred = model(img, augment=opt.augment)[0]
        t2 = time_synchronized()

        # Apply NMS
        pred = non_max_suppression(pred, opt.conf_thres, opt.iou_thres, classes=opt.classes, agnostic=opt.agnostic_nms)
        t3 = time_synchronized()

        # Apply Classifier
        if classify:
            pred = apply_classifier(pred, modelc, img, im0s)

        """
        # pred 값 확인: (x, y, x1, y1, 정확도, class)가 하나의 box이고 이런 box 여러 개인 tensor
        # print(pred)
        
        # 원본 이미지 띄우기
        # print("image check!!!!!!!!!!!")
        # temp=Image.fromarray(im0s)
        # temp.show()
        
        # tensor의 값 확인
        # print(pred[0][0])
        # print(pred[0][0][0])
        
        # tensor float에서 int로 만드려는 시도 -> 실패
        # x1=torch.IntTensor(pred[0][0][0])
        # x2=torch.IntTensor(pred[0][0][2])
        # y1=torch.IntTensor(pred[0][0][1])
        # y2=torch.IntTensor(pred[0][0][3])
        
        # x1=(pred[0][0][0]).type(torch.IntTensor)
        # x2=(pred[0][0][2]).type(torch.IntTensor)
        # y1=(pred[0][0][1]).type(torch.IntTensor)
        # y2=(pred[0][0][3]).type(torch.IntTensor)
        """
        
        # 원본 이미지 저장. 자른 이미지에 bounding box 침범 문제 해결 위함
        test_img=copy(im0s)
        
        # 각 box의 color 히스토그램 리스트에 저장
        redhist_list=[]
        bluehist_list=[]
        greenhist_list=[]
        
        # frame당 박스 중앙 좌표 저장한 리스트
        centercoord_list=[]
        
        #frame당 박스 이미지 저장하는 리스트
        frameboximage_list=[]
        
        # 박스 색 변화 위한 리스트 
        c_color_list=[]
        
        # frame의 여러 box 있는 것 loop 돌리기
        # Process detections
        for i, det in enumerate(pred):  # detections per image
            if webcam:  # batch_size >= 1
                p, s, im0, frame = path[i], '%g: ' % i, im0s[i].copy(), dataset.count
            else:
                p, s, im0, frame = path, '', im0s, getattr(dataset, 'frame', 0)
            
            p = Path(p)  # to Path
            save_path = str(save_dir / p.name)  # img.jpg
            txt_path = str(save_dir / 'labels' / p.stem) + ('' if dataset.mode == 'image' else f'_{frame}')  # img.txt
            gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
            
            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()
                # print(det)

                # Print results
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()  # detections per class
                    s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "  # add to string

                # Write results
                for *xyxy, conf, cls in reversed(det):
                    
                    x1=xyxy[0].type(torch.IntTensor)
                    x2=xyxy[2].type(torch.IntTensor)
                    y1=xyxy[1].type(torch.IntTensor)
                    y2=xyxy[3].type(torch.IntTensor)

                    # box에 중간 좌표 계산 후 저장
                    center_x=(x1.item()+x2.item())/2
                    center_y=(y1.item()+y2.item())/2
                    center_position=position(center_x,center_y)
                    centercoord_list.append(center_position)
                    
                    """
                    # # box에 중간 좌표 계산 후 저장
                    # box_centercoord_list=[]
                    # box_centercoord_list.append(center_x)
                    # box_centercoord_list.append(center_y)
                    # centercoord_list.append(box_centercoord_list)
                    """
                    
                    """
                    # print(x1.item()," ",x2.item()," ",y1.item()," ",y2.item())
                    # print(box_centercoord_list)
                    
                    # box 자르기
                    # 바로 pred[0][0][1]하면 float라 안됨
                    # img_trim=im0[y1+1:y2+1,x1+1:x2+1]
                    
                    # 이미지에서 box 안 보이게 하기 위함
                    # img_trim=test_img[y1+1:y2+1,x1+1:x2+1]
                    """
                    
                    # 영상이나 웹캠 하려면 이렇게 해야함
                    img_trim=im0[y1+1:y2+1,x1+1:x2+1]
                    
                    """
                    # 컬러 히스토그램 
                    # histr=cv2.calcHist([img_trim],channels=[1, 2], mask=None, histSize=[32, 32], ranges=[0,256, 0, 256])
                    # histr=cv2.calcHist([img_trim],channels=[0], mask=None, histSize=[256], ranges=[0,256])
                    # histr /= histr.sum() # normalize sum to 1
                    # print(histr)
                    """
                    
                    """
                    # 리스트에 중심 x 좌표와 히스토그램 or image 저장
                    # 왜냐하면 추후에 sort에 사용하기 위함
                    red_list=[]
                    blue_list=[]
                    green_list=[]
                    image_list=[]
                    
                    # red 히스토그램
                    histrR=cv2.calcHist([img_trim],channels=[2], mask=None, histSize=[256], ranges=[0,256])
                    histrR /= histrR.sum() # normalize sum to 1
                    red_list.append(center_x)
                    red_list.append(histrR)
                    redhist_list.append(red_list)
                    # redhist_list.append(histrR)
                    
                    # green 히스토그램
                    histrG=cv2.calcHist([img_trim],channels=[1], mask=None, histSize=[256], ranges=[0,256])
                    histrG /= histrG.sum() # normalize sum to 1
                    green_list.append(center_x)
                    green_list.append(histrG)
                    greenhist_list.append(green_list)
                    # greenhist_list.append(histrG)
                    
                    # blue 히스토그램
                    histrB=cv2.calcHist([img_trim],channels=[0], mask=None, histSize=[256], ranges=[0,256])
                    histrB /= histrB.sum() # normalize sum to 1
                    blue_list.append(center_x)
                    blue_list.append(histrB)
                    bluehist_list.append(blue_list)
                    # bluehist_list.append(histrB)
                    """

                    # 리스트에 중심 x 좌표와 히스토그램 or image 저장
                    # 왜냐하면 추후에 sort에 사용하기 위함

                    # red 히스토그램
                    histrR=cv2.calcHist([img_trim],channels=[2], mask=None, histSize=[256], ranges=[0,256])
                    histrR /= histrR.sum() # normalize sum to 1
                    red_histogram=histogram_class(histrR,center_x)
                    redhist_list.append(red_histogram)
                    
                    # green 히스토그램
                    histrG=cv2.calcHist([img_trim],channels=[1], mask=None, histSize=[256], ranges=[0,256])
                    histrG /= histrG.sum() # normalize sum to 1
                    green_histogram=histogram_class(histrG,center_x)
                    greenhist_list.append(green_histogram)
                    
                    # blue 히스토그램
                    histrB=cv2.calcHist([img_trim],channels=[0], mask=None, histSize=[256], ranges=[0,256])
                    histrB /= histrB.sum() # normalize sum to 1
                    blue_histogram=histogram_class(histrB,center_x)
                    bluehist_list.append(blue_histogram)
                    
                    """
                    # 이미지 보여주기
                    # cv2.imwrite('temp1.jpg',img_trim)
                    # org_image=cv2.imread('temp1.jpg')
                    # cv2.imshow('org_image',org_image)
                    # cv2.waitKey(0)
                    """
                    
                    image_list=[]

                    # 박스 이미지를 리스트에 저장
                    # frameboximage_list.append(org_image)
                    image_list.append(center_x)
                    image_list.append(img_trim)
                    frameboximage_list.append(image_list)
                    # frameboximage_list.append(img_trim)
                    
                    # 자른 이미지 저장
                    # cv2.imwrite('E:/choij/TerminalTech/LaurentKoscielny/runs/photo/img'+str(image_num)+'.jpeg', img_trim)
                    # image_num=image_num+1
                    
                    # box 하나 히스토그램 저장할 때마다 cnt ++
                    cnt=cnt+1
                    # print(cnt)
                    
                    if save_txt:  # Write to file
                        xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
                        line = (cls, *xywh, conf) if opt.save_conf else (cls, *xywh)  # label format
                        with open(txt_path + '.txt', 'a') as f:
                            f.write(('%g ' * len(line)).rstrip() % line + '\n')

                    if save_img or view_img:  # Add bbox to image
                        label = f'{names[int(cls)]} {conf:.2f}'
                        # print(xyxy)
                        # if color_cnt==0:
                        #     plot_one_box(xyxy, im0, label=label, color=colors[int(cls)], line_thickness=1)
                        # else:
                            # plot_one_box(xyxy, im0, label=label, color=(0,0,255), line_thickness=1)
                        
                        plot_one_box(xyxy, im0, label=label, color=colors[int(cls)], line_thickness=1)
                        
                        color_list=[]
                        color_list.append(xyxy)
                        color_list.append(im0)
                        color_list.append(label)
                        c_color_list.append(color_list)

            # Print time (inference + NMS)
            # print(f'{s}Done. ({(1E3 * (t2 - t1)):.1f}ms) Inference, ({(1E3 * (t3 - t2)):.1f}ms) NMS')

            """
            # -to do list- (해결)
            # box가 찾아지는 순서가 달라서 전 frame과 배열로 비교하면 순서 안 맞는 경우가 생김
            # 좌표를 key로 하고 red, green, blue, image를 value로 해서 map으로 저장해두고 list sorting해서 큰 list에 저장
            # 이렇게 해도 ex: 3개 box는 움직이지 않고 1개가 움직이면 1 frame 때는 A, B, C, D로 저장되고 6 frame 때는 B, C, D, A로 저장될 수 있다.
            # 그래도 5 frame 만에 좌표 100씩 이동하고 그렇지 않지 않을까?
            """

            # redhist_list.sort(key = lambda x:x[0])
            # greenhist_list.sort(key = lambda x:x[0])
            # bluehist_list.sort(key = lambda x:x[0])
            frameboximage_list.sort(key = lambda x:x[0])
            # centercoord_list.sort(key = lambda x:x[0])

            redhist_list.sort(key = lambda histogram_class:histogram_class.center_x)
            greenhist_list.sort(key = lambda histogram_class:histogram_class.center_x)
            bluehist_list.sort(key = lambda histogram_class:histogram_class.center_x)
            centercoord_list.sort(key=lambda position:position.x)

            # frame별 히스토그램 및 박스 중앙 좌표 및 박스 이미지 저장
            redhist.append(redhist_list)
            bluehist.append(bluehist_list)
            greenhist.append(greenhist_list)
            centercoord.append(centercoord_list)
            frameboximage.append(frameboximage_list)
            
            change_color_list.append(c_color_list)
            
            # 이전 frame과 히스토그램 비교 로직
            
            # 비교 알고리즘의 이름들을 리스트에 저장
            methods = ['CORREL', 'CHISQR', 'INTERSECT', 'BHATTACHARYYA', 'EMD']
     
            # n번째 frame부터 비교 시작
            if framenum>=beforeframe:
                if framenum%beforeframe==0:
                    # print()
                    # print("-----------------------------------------------------")
                    # print(framenum+1,"번째 frame")
                    
                    # 각 frame에 감지한 box 개수가 같으면 실행
                    if len(redhist[framenum])==len(redhist[framenum-beforeframe]):
                        
                        """
                        # to do list (23.08.17) (해결)
                        # 같은 박스 개수여도 좌표 아예 차이 많이 나는 것은 비교하면 안됨
                        # 더 나아가 같은 박스를 찾아서 비교하도록 해야 함
                        #100, 119, 170, 202번째 frame에서 문제 발생
                        """

                        # print()
                        # print("감지한 box 개수: ", len(redhist[framenum]))
                        # print()
                            
                        # print("이번 frame 히스토그램 개수: ", len(redhist[framenum]))
                        # print("전 frame 히스토그램 개수: ", len(redhist[framenum-beforeframe]))
                        
                        # print("현재 frame box의 전체 이미지 중앙 좌표: ", centercoord[framenum])
                        # print("n번째 전 frame box의 전체 이미지 중앙 좌표: ", centercoord[framenum-beforeframe])
                        # print()

                        # print("이번 frame box 전체 좌표: ")
                        # for thisframecoord in centercoord[framenum]:
                        #     print("[",thisframecoord.x,",",thisframecoord.y,"]")
                        
                        # print()
                        
                        # print("전 frame box 전체 좌표: ")
                        # for beforeframecoord in centercoord[framenum-beforeframe]:
                        #     print("[",beforeframecoord.x,",",beforeframecoord.y,"]")
                        
                        # print()
                        
                        
                        # 두번째 frame에 있는 box마다 히스토그램 비교
                        for now_box in range(len(redhist[framenum])):
                            for before_box in range(len(redhist[framenum])):
                            
                                # if abs(centercoord[framenum][now_box][0]-centercoord[framenum-beforeframe][j][0]) < coordrate:
                                if abs(centercoord[framenum][now_box].x-centercoord[framenum-beforeframe][before_box].x) < coordrate:
                                        # 어떤 히스토그램 차이를 기준으로 변화 감지할 것인지
                                        comparenum=0
                                        
                                        # print("현재 frame 좌표: ", centercoord[framenum][i][0])
                                        # print("이전 frame 좌표: ", centercoord[framenum-beforeframe][j][0])
                                        # print("현재 frame 좌표: [", centercoord[framenum][now_box].x,",",centercoord[framenum][now_box].y,"]")
                                        # print("이전 frame 좌표: [", centercoord[framenum-beforeframe][before_box].x,",",centercoord[framenum-beforeframe][before_box].y,"]")
                                    
                                        """
                                    # 5회 반복(5개 비교 알고리즘을 모두 사용)
                                    # for index, name in enumerate(methods):
                                            
                                        # 비교 알고리즘 이름 출력(문자열 포맷팅 및 탭 적용)
                                        # print('%-10s' % name, end = '\n')
                                            
                                        # 5개 다 돌릴 때 사용하는 것
                                        # ret1 = cv2.compareHist(redhist[framenum][i][1], redhist[framenum-beforeframe][j][1], index)
                                        # ret2 = cv2.compareHist(greenhist[framenum][i][1], greenhist[framenum-beforeframe][j][1], index)
                                        # ret3 = cv2.compareHist(bluehist[framenum][i][1], bluehist[framenum-beforeframe][j][1], index)
                                        """    
                                    
                                        # ret1 = cv2.compareHist(redhist[framenum][now_box][1], redhist[framenum-beforeframe][j][1], 3)
                                        # ret2 = cv2.compareHist(greenhist[framenum][now_box][1], greenhist[framenum-beforeframe][j][1], 3)
                                        # ret3 = cv2.compareHist(bluehist[framenum][now_box][1], bluehist[framenum-beforeframe][j][1], 3)
                                        ret1 = cv2.compareHist(redhist[framenum][now_box].histogram, redhist[framenum-beforeframe][before_box].histogram, 3)
                                        ret2 = cv2.compareHist(greenhist[framenum][now_box].histogram, greenhist[framenum-beforeframe][before_box].histogram, 3)
                                        ret3 = cv2.compareHist(bluehist[framenum][now_box].histogram, bluehist[framenum-beforeframe][before_box].histogram, 3)
                                            
                                        """
                                        # if index==3:
                                        #     comparenum=ret2
                                        
                                        # comparenum=ret2
                                            
                                        # # 교차 분석인 경우
                                        # if index == cv2.HISTCMP_INTERSECT:
                                        #     # 원본으로 나누어 1로 정규화
                                        #     ret1 = ret1/np.sum(redhist[framenum][i][1])
                                        #     ret2 = ret2/np.sum(greenhist[framenum][i][1])
                                        #     ret3 = ret3/np.sum(bluehist[framenum][i][1])
                                        """
                                        
                                        ret1 = ret1/np.sum(redhist[framenum][now_box].histogram)
                                        ret2 = ret2/np.sum(greenhist[framenum][now_box].histogram)
                                        ret3 = ret3/np.sum(bluehist[framenum][now_box].histogram)
                                        
                                        # 비교 결과 출력
                                        # print("이전 frame과 red 히스토그램 결과 비교 :", (ret1), end='\n')
                                        # print("이전 frame과 green 히스토그램 결과 비교 :", (ret2), end='\n')
                                        # print("이전 frame과 blue 히스토그램 결과 비교 :", (ret3), end='\n')
                                        # print()
                                        
                                        # if comparenum>=detectrate:
                                        if ret1>=detectrate and ret2>=detectrate and ret3>=detectrate:
                                            
                                            print("change detect")
                                            
                                            red_color=(0,0,255)
                                            
                                            # 바뀐 박스 빨간색으로 바뀌는 것
                                            # plot_one_box(change_color_list[framenum][now_box][0], change_color_list[framenum][now_box][1], label=change_color_list[framenum][now_box][2], color=red_color, line_thickness=1)
                                            
                                            color_cnt=500
                                            
                                            # 사진 크기 크게 고정한 것
                                            # org_img= cv2.resize(frameboximage[framenum][now_box][1], dsize=(300,300), fx=1, fy=1, interpolation=cv2.INTER_LINEAR)
                                            # tenframe_img= cv2.resize(frameboximage[framenum-beforeframe][before_box][1], dsize=(300,300), fx=1, fy=1, interpolation=cv2.INTER_LINEAR)

                                            # EXAMPLE
                                            # TODO publish set in here
                                            # coord = Pose2D()
                                            # coord.x = x
                                            # coord.y = y
                                            # coord.theta = 0 # Do not change
                                            # coord_pub.publish(coord)
                                            send_coord = Pose2D()
                                            send_coord.x = centercoord[framenum][now_box].x
                                            send_coord.y = centercoord[framenum][now_box].y
                                            send_coord.theta = 0
                                            coord_pub.publish(send_coord)
                                            
                                            # print("현재 frame 좌표: [", centercoord[framenum][now_box].x,",",centercoord[framenum][now_box].y,"]")
                                            # print("이전 frame 좌표: [", centercoord[framenum-beforeframe][before_box].x,",",centercoord[framenum-beforeframe][before_box].y,"]")
                                            
                                            detect_cnt=detect_cnt+1
                                            
                                            org_img=frameboximage[framenum][now_box][1]
                                            tenframe_img=frameboximage[framenum-beforeframe][before_box][1]
                                            
                                            # cv2.imshow('org_image',org_img)
                                            # cv2.imshow('tenframe_image',tenframe_img)
                                            # cv2.waitKey(0)
                                                
                                            # print("현재 frame 이미지 중앙 좌표: ", centercoord[framenum][now_box])
                                            # print("n번째 전 frame 이미지 중앙 좌표: ", centercoord[framenum-beforeframe][now_box])
                                        
                                        # centercoord[framenum-beforeframe][j][0]=-1*centercoord[framenum-beforeframe][j][0]
                                        centercoord[framenum-beforeframe][before_box].x=-1*centercoord[framenum-beforeframe][before_box].x
                                        break
                                # else:
                                #     print("---다른 박스---")
                                    
                                
                        for inx in range(len(redhist[framenum])):
                            # if centercoord[framenum-beforeframe][inx][0]<0:
                            #     centercoord[framenum-beforeframe][inx][0]=-1*centercoord[framenum-beforeframe][inx][0]
                            if centercoord[framenum-beforeframe][inx].x<0:
                                centercoord[framenum-beforeframe][inx].x=-1*centercoord[framenum-beforeframe][inx].x
                            
                    else:
                        # print("\n히스토그램 개수 다름!!!!!!!!!!!!!!!\n") 
                        
                        # 개수 많은 박스 개수 저장 변수
                        longhistlen=0
                        
                        # 개수 적은 박스 개수 저장 변수
                        shorthistlen=0
                        
                        # 긴 좌표 리스트, 짧은 좌표 리스트 저장 변수
                        many_box_frame_coord_list=[]
                        less_box_frame_coord_list=[]
                        
                        # 더 쉽게 사용하기 위해 박스 개수 많은 frame과 적은 frame 번호를 변수로 사용
                        longframe=0
                        shortframe=0
                        
                        if len(redhist[framenum])>len(redhist[framenum-beforeframe]):
                            # print("새로운 box 생성됨.\n")
                            
                            longframe=framenum
                            shortframe=framenum-beforeframe

                        else:
                            # print("box 사라짐.\n")
                            
                            longframe=framenum-beforeframe
                            shortframe=framenum
                            
                        longhistlen=len(redhist[longframe])
                        shorthistlen=len(redhist[shortframe])
                            
                        many_box_frame_coord_list=centercoord[longframe]
                        less_box_frame_coord_list=centercoord[shortframe]
                        
                        # print("이번 frame 히스토그램 개수: ", len(redhist[framenum]))
                        # print("전 frame 히스토그램 개수: ", len(redhist[framenum-beforeframe]))
                        # print()
                        
                        # print("이번 frame box 전체 좌표: ")
                        # for thisframecoord in centercoord[framenum]:
                        #     print("[",thisframecoord.x,",",thisframecoord.y,"]")
                        
                        # print()
                        
                        # print("전 frame box 전체 좌표: ")
                        # for beforeframecoord in centercoord[framenum-beforeframe]:
                        #     print("[",beforeframecoord.x,",",beforeframecoord.y,"]")
                        
                        # print()
                        
                        """
                        # for temp1 in frameboximage[framenum]:
                        #     cv2.imshow('test1',temp1[1])
                        #     cv2.waitKey(500) 
                        
                        # for temp2 in frameboximage[framenum-beforeframe]:
                        #     cv2.imshow('test2',temp2[1])
                        #     cv2.waitKey(500) 
                        
                        
                        # for temp in range(longhistlen):
                        #     if temp<len(frameboximage[framenum]):
                        #         # cv2.imshow('this_frame_box_image', frameboximage[framenum][temp][1])
                        #         print("현재 frame box 중앙 x 좌표: ", frameboximage[framenum][temp][0])
                        #         # cv2.waitKey(0)
                            
                        #     if temp<len(frameboximage[framenum-beforeframe]):
                        #         # cv2.imshow('before_frame_box_image', frameboximage[framenum-beforeframe][temp][1])
                        #         print("이전 frame box 중앙 x 좌표: ", frameboximage[framenum-beforeframe][temp][0])
                        #         # cv2.waitKey(0)
                        """
                        
                        """
                        # to do list (23.08.17) (완료)
                        # 박스 같은 것인지 비교하는데 끝까지 같은 것 못 찾으면 버리고 다음 박스 비교하도록 해야 함
                        # 현재 frame에서 하나 box 새로 생기고 두개 box 사라지면 오류남
                        # 2중 for문으로 다 돌려봄. 이것도 약간은 문제가 있는 것 같기도 함
                        """

                        # to do list (23.08.22)
                        # 83, 84 or 85, 130, 135, 141 frame 문제 발생
                        # box 개수 2개 차이나면 뭔가 문제 생기는게 몇개 있다.
                        # 135, 141 frame의 문제는 겹치는 부분에서 문제가 생기는 것 같다. y 좌표까지 같이 사용하면 괜찮을 듯 싶다.
                        
                        for less_box in range(shorthistlen):
                            for many_box in range(longhistlen):
                                # 두 박스의 x 좌표의 차이를 구해서 50 이하인지 확인
                                # if abs(less_box_frame_coord_list[less_box][0]-many_box_frame_coord_list[many_box][0]) < coordrate:
                                if abs(less_box_frame_coord_list[less_box].x-many_box_frame_coord_list[many_box].x) < coordrate:
                                    # print("같은 박스")
                                    # print("좌표: [", less_box_frame_coord_list[less_box].x,",", less_box_frame_coord_list[less_box].y,"] [", many_box_frame_coord_list[many_box].x,",",many_box_frame_coord_list[many_box].y,"]")
                                    
                                    # 같은 박스끼리 히스토그램 비교
                                    # ret1 = cv2.compareHist(redhist[longframe][many_box][1], redhist[shortframe][less_box][1], 3)
                                    # ret2 = cv2.compareHist(greenhist[longframe][many_box][1], greenhist[shortframe][less_box][1], 3)
                                    # ret3 = cv2.compareHist(bluehist[longframe][many_box][1], bluehist[shortframe][less_box][1], 3)

                                    ret1 = cv2.compareHist(redhist[longframe][many_box].histogram, redhist[shortframe][less_box].histogram, 3)
                                    ret2 = cv2.compareHist(greenhist[longframe][many_box].histogram, greenhist[shortframe][less_box].histogram, 3)
                                    ret3 = cv2.compareHist(bluehist[longframe][many_box].histogram, bluehist[shortframe][less_box].histogram, 3)
                                    
                                    # 비교 결과 출력
                                    # print("이전 frame과 red 히스토그램 결과 비교 :", (ret1), end='\n')
                                    # print("이전 frame과 green 히스토그램 결과 비교 :", (ret2), end='\n')
                                    # print("이전 frame과 blue 히스토그램 결과 비교 :", (ret3), end='\n')
                                    # print()
                                    
                                    comparenum=ret2
                                    
                                    # if comparenum>=detectrate:
                                    if ret1>=detectrate and ret2>=detectrate and ret3>=detectrate:
                                        print("change detect")
                                        
                                        # shortframe_image= cv2.resize(frameboximage[shortframe][less_box][1], dsize=(300,300), fx=1, fy=1, interpolation=cv2.INTER_LINEAR)
                                        # longframe_image= cv2.resize(frameboximage[longframe][many_box][1], dsize=(300,300), fx=1, fy=1, interpolation=cv2.INTER_LINEAR)
                                        
                                        if len(redhist[framenum])>len(redhist[framenum-beforeframe]):

                                            # EXAMPLE
                                            # TODO publish set in here
                                            # coord = Pose2D()
                                            # coord.x = x
                                            # coord.y = y
                                            # coord.theta = 0 # Do not change
                                            # coord_pub.publish(coord)
                                            send_coord = Pose2D()
                                            send_coord.x = many_box_frame_coord_list[many_box].x
                                            send_coord.y = many_box_frame_coord_list[many_box].y
                                            send_coord.theta = 0
                                            coord_pub.publish(send_coord)

                                            # print("현재 frame 좌표: [", many_box_frame_coord_list[many_box].x,",",many_box_frame_coord_list[many_box].y,"]")
                                            # print("이전 frame 좌표: [", less_box_frame_coord_list[less_box].x,",",less_box_frame_coord_list[less_box].y,"]")
                                            
                                        else:
                                            # EXAMPLE
                                            # TODO publish set in here
                                            # coord = Pose2D()
                                            # coord.x = x
                                            # coord.y = y
                                            # coord.theta = 0 # Do not change
                                            # coord_pub.publish(coord)
                                            send_coord = Pose2D()
                                            send_coord.x = less_box_frame_coord_list[less_box].x
                                            send_coord.y = less_box_frame_coord_list[less_box].y
                                            send_coord.theta = 0
                                            coord_pub.publish(send_coord)

                                            # print("현재 frame 좌표: [", less_box_frame_coord_list[less_box].x,",",less_box_frame_coord_list[less_box].y,"]")
                                            # print("이전 frame 좌표: [", many_box_frame_coord_list[many_box].x,",",many_box_frame_coord_list[many_box].y,"]")
                                        
                                        detect_cnt=detect_cnt+1
                                        
                                        shortframe_image=frameboximage[shortframe][less_box][1]
                                        longframe_image=frameboximage[longframe][many_box][1]
                                    
                                        # cv2.imshow('shortframe_image',shortframe_image) 
                                        # cv2.imshow('longframe_image',longframe_image)
                                        # cv2.waitKey(0)
                                    
                                    # many_box_frame_coord_list[many_box][0]=-1*many_box_frame_coord_list[many_box][0]
                                    many_box_frame_coord_list[many_box].x=-1*many_box_frame_coord_list[many_box].x
                                    break
                                
                                # else:
                                    # print("다른 박스")
                                
                        for box in range(longhistlen):
                            # if many_box_frame_coord_list[long][0]<0:
                            #     many_box_frame_coord_list[long][0]=-1*many_box_frame_coord_list[long][0]
                            if many_box_frame_coord_list[box].x<0:
                                many_box_frame_coord_list[box].x=-1*many_box_frame_coord_list[box].x
               
                        """
                        # x 좌표 차이 50 이하인 것들을 같은 박스로 생각하고 둘의 히스토그램을 비교하는 것
                        many_box=0
                        less_box=0
                        
                        while less_box<shorthistlen:
                            
                            # 두 박스의 x 좌표의 차이를 구해서 50 이하인지 확인
                            if abs(less_box_frame_coord_list[less_box][0]-many_box_frame_coord_list[many_box][0]) < coordrate:
                                print("같은 박스")
                                print("좌표: ", less_box_frame_coord_list[less_box]," ", many_box_frame_coord_list[many_box])
                                
                                # 같은 박스끼리 히스토그램 비교
                                ret1 = cv2.compareHist(redhist[longframe][many_box][1], redhist[shortframe][less_box][1], 3)
                                ret2 = cv2.compareHist(greenhist[longframe][many_box][1], greenhist[shortframe][less_box][1], 3)
                                ret3 = cv2.compareHist(bluehist[longframe][many_box][1], bluehist[shortframe][less_box][1], 3)
                                
                                # 비교 결과 출력
                                print("이전 frame과 red 히스토그램 결과 비교 :", (ret1), end='\n')
                                print("이전 frame과 green 히스토그램 결과 비교 :", (ret2), end='\n')
                                print("이전 frame과 blue 히스토그램 결과 비교 :", (ret3), end='\n')
                                print()
                                
                                comparenum=ret2
                                
                                if comparenum>=detectrate:
                                    print("change detect")
                                    
                                    shortframe_image= cv2.resize(frameboximage[shortframe][less_box][1], dsize=(0,0), fx=5, fy=5, interpolation=cv2.INTER_LINEAR)
                                    longframe_image= cv2.resize(frameboximage[longframe][many_box][1], dsize=(0,0), fx=5, fy=5, interpolation=cv2.INTER_LINEAR)
                                
                                    cv2.imshow('shortframe_image',shortframe_image) 
                                    cv2.imshow('longframe_image',longframe_image)
                                    cv2.waitKey(0)
                                    
                                less_box=less_box+1
                            
                            else:
                                print("다른 박스")
                            
                            print()
                            many_box=many_box+1
                        """
                        
                        """
                        # to do list (해결)
                        # less_box_frame_coord_list, many_box_frame_coord_list를 리스트로 저장하는 것이 아닌 framenum, framenum-beforenum을 저장하게 해서 
                        # 여기서 centercoord[less_box_frame_coord_list][less_box][0] 이렇게 쓰게 하는 것이다. 
                        # 그 후 같은 박스 찾은 후 히스토그램 비교 시에 redhist[less_box_frame_coord_list][less_box][1], redhist[many_box_frame_coord_list][many_box][1] 이렇게 사용하면 될 듯하다.
                        """
                            
                            
                    # print("-----------------------------------------------------")
            
            framenum=framenum+1
            color_cnt=color_cnt-1
            
            # Stream results
            if view_img:
                cv2.imshow(str(p), im0)
                cv2.waitKey(1)  # 1 millisecond

            # Save results (image with detections)
            if save_img:
                if dataset.mode == 'image':
                    cv2.imwrite(save_path, im0)
                    # print(f" The image with the result is saved in: {save_path}")
                else:  # 'video' or 'stream'
                    if vid_path != save_path:  # new video
                        vid_path = save_path
                        if isinstance(vid_writer, cv2.VideoWriter):
                            vid_writer.release()  # release previous video writer
                        if vid_cap:  # video
                            fps = vid_cap.get(cv2.CAP_PROP_FPS)
                            w = int(vid_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                            h = int(vid_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                        else:  # stream
                            fps, w, h = 30, im0.shape[1], im0.shape[0]
                            save_path += '.mp4'
                        vid_writer = cv2.VideoWriter(save_path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
                    vid_writer.write(im0)

        # 실시간 몇 frame일 때 종료할 것인지
        # if framenum==100:
        #     break
        
        key = cv2.waitKey(1)

        # 종료 조건 체크
        if key == ord('q'):
            break
    
    if save_txt or save_img:
        s = f"\n{len(list(save_dir.glob('labels/*.txt')))} labels saved to {save_dir / 'labels'}" if save_txt else ''
        #print(f"Results saved to {save_dir}{s}")

    # print(f'Done. ({time.time() - t0:.3f}s)')
    
    # 전체 히스토그램 개수 출력
    # print()
    # print("전체 히스토그램 개수: " , cnt)
    # print("= 탐지된 box 개수\n")
    
    """
    # 전체 히스토그램 개수 맞는지 확인 위한 값. 
    # 아래 2중 for문에서 돌면서 확인함 
    # count=0
    
    # exp 폴더에 히스토그램 저장하기 위한 지정
    # directory = str(save_dir)
    # filename_format = "colorHistogram{}.png" 
    
    # 히스토그램 그리고 저장
    
    # color=('b','g','r')
    # for red, blue, green in zip(redhist,bluehist,greenhist) :
    #     onefor=0
    #     # for i in range (0, len(bluehist_list)) :
    #     for i in range (0, len(red)) :

    #         # plt.plot(bluehist_list[i], color='b')
    #         # plt.plot(greenhist_list[i], color='g')
    #         # plt.plot(redhist_list[i], color='r')
    #         # plt.xlim([0,256])
    #         # # filename=f"colorHistogram_{i}.png"
    #         # # plt.savefig(filename)
    #         # plt.show()
            
    #         plt.plot(blue[i], color='b')
    #         plt.plot(green[i], color='g')
    #         plt.plot(red[i], color='r')
    #         plt.xlim([0,256])
    #         # filename=f"colorHistogram_{i}.png"
    #         # plt.savefig(filename)
    #         filename=filename_format.format(i)
    #         filepath = os.path.join(directory, filename)
    #         plt.savefig(filepath)
    #         plt.show()
    #         onefor=onefor+1
    #     # print(onefor)
    #     count=count+onefor
    
    # 전체 히스토그램 저장 되었는지 확인
    # print(count)
    """
    
    # frame 개수 출력 
    # = frame 당 각 box에 히스토그램 저장된 리스트 저장되었는지 확인 
    # print("red 히스토그램 개수: ", len(redhist))
    # print("blue 히스토그램 개수: ",len(bluehist))
    # print("green 히스토그램 개수: ",len(greenhist))
    # print("= 전체 frame 개수\n")
    
    # 전체 히스토그램 개수 맞는지 확인
    testcnt=0
    for histlist in redhist:
        testcnt=testcnt+len(histlist)
    
    # print("전체 히스토그램 개수: ",testcnt)
    
    # 각 frame에 리스트 저장 되었는지 확인
    # print("frame마다 저장된 x,y 좌표 리스트 개수: ", len(centercoord))
    
    # 전체 x, y 좌표 개수 확인
    testcnt2=0
    for center in centercoord:
        testcnt2=testcnt2+len(center)

    # EXAMPLE
    # TODO publish set in here
    # coord = Pose2D()
    # coord.x = x
    # coord.y = y
    # coord.theta = 0 # Do not change
    # coord_pub.publish(coord)
    
    # print("전체 x,y 좌표 개수: ", testcnt2)
    # print()
    
    # print("change detect된 횟수: ",detect_cnt)
    

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', nargs='+', type=str, default=(str(pathlib.Path(__file__).parent.absolute()) + '/my-tiny.pt'), help='model.pt path(s)')
    parser.add_argument('--source', type=str, default='0', help='source')  # file/folder, 0 for webcam
    parser.add_argument('--img-size', type=int, default=640, help='inference size (pixels)')
    parser.add_argument('--conf-thres', type=float, default=0.25, help='object confidence threshold')
    parser.add_argument('--iou-thres', type=float, default=0.45, help='IOU threshold for NMS')
    parser.add_argument('--device', default='', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    parser.add_argument('--view-img', action='store_true', help='display results')
    parser.add_argument('--save-txt', action='store_true', help='save results to *.txt')
    parser.add_argument('--save-conf', action='store_true', help='save confidences in --save-txt labels')
    parser.add_argument('--nosave', action='store_true', help='do not save images/videos')
    parser.add_argument('--classes', nargs='+', type=int, help='filter by class: --class 0, or --class 0 2 3')
    parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
    parser.add_argument('--augment', action='store_true', help='augmented inference')
    parser.add_argument('--update', action='store_true', help='update all models')
    parser.add_argument('--project', default='runs/detect', help='save results to project/name')
    parser.add_argument('--name', default='exp', help='save results to project/name')
    parser.add_argument('--exist-ok', action='store_true', help='existing project/name ok, do not increment')
    parser.add_argument('--no-trace', action='store_true', help='don`t trace model')
    opt = parser.parse_args()
    print(opt)
    #check_requirements(exclude=('pycocotools', 'thop'))

    rospy.init_node('detect_yolov7', anonymous=True)
    coord_pub = rospy.Publisher("changed_coordinate_from_image", Pose2D, queue_size=10)

    with torch.no_grad():
        if opt.update:  # update all models (to fix SourceChangeWarning)
            for opt.weights in ['yolov7.pt']:
                detect()
                strip_optimizer(opt.weights)
        else:
            detect()