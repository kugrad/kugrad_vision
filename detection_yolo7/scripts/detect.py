#! /usr/bin/python3
import rospy
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import torch
import numpy as np
from numpy import random

from models.experimental import attempt_load
from utils.torch_utils import select_device, load_classifier, time_synchronized, TracedModel
from utils.datasets import letterbox
from utils.general import non_max_suppression

MODEL_NAME      = "model_name"  # ex) tiny.pt
OBJ_CONFTHRES   = 0.25          # object confidence threshold
IOU_THRES       = 0.45          # iou threshold
DEVICE          = ""            # device cuda or something else
ARGUMENT        = False         # yolov7 model's arugment value / if you had setting --arugment in yolov7, this var must be set as True and vice versa...

class DetectYolo:
    def __init__(self):
        global MODEL_NAME
        global OBJ_CONFTHRES

        self.bridge = CvBridge()

        # initialize
        self.device = select_device('')
        self.half = self.device.type != 'cpu'
        self.weight = MODEL_NAME
        self.conf_thres = OBJ_CONFTHRES
        self.iou_thres =  IOU_THRES
        self.device = DEVICE
        self.argument = ARGUMENT

        # load model
        self.model = attempt_load(self.weight, map_location=self.device)
        self.stride = int(self.model.stride.max())

        if self.half:
            self.model.half()

        # Get names and colors
        self.names = self.model.module.names if hasattr(self.model, 'module') else self.model.names
        self.colors = [[random.randint(0, 255) for _ in range(3)] for _ in self.names]

        # 여기에 있는 배열의 각 저장된 하나씩은 frame의 정보다. 
        # 동영상 용 frame 별 각 box 당 히스토그램 저장 위한 list
        self.redhis, self.bluehis, self.greenhis = [], [], []

        # frame 당 중앙박스 좌표 저장 리스트, frame 당 박스 이미지 저장 리스트,
        self.center_coord, self.frame_box_img= [], []

        # 현재 detect 되는 히스토그램 개수
        self.cnt = 0

        # 히스토그램 비교 로직에서 현재 프레임이 몇번째 프레임인지, 몇번째 frame 전과 비교할 것인가 저장
        self.frame_num = 0
        self.before_frame = 10

        # 변화 검출 비율
        self.detect_rate = 0.1

        # 박스가 같은 박스인지 확인시 좌표 차이
        self.coord2chk_same_box = 15

        # (test 기능 구현) change detct시 박스 색 바꾸기 
        # xyxy, im0 저장 위한 리스트
        self.change_color_list=[]
    
        # box 색 유지 위한 변수
        self.color_cnt=0

        self.t0 = rospy.Time().now()

    def initRos(self):
        rospy.init_node("detect_yolo7", anonymous=True)
        rospy.Subscriber("stereo/left_image", Image, self.receiveImageCallback)
        rospy.spin()
    
    def receiveImageCallback(self, image_data: Image):
        # image => received left camera image
        try:
            self.img0 = self.bridge.imgmsg_to_cv2(image_data, image_data.encoding)
        except CvBridgeError as e:
            rospy.INFO("Error convert image %s", e) 
            return

        height, width = self.img0.shape
        self.image_size = (width, height)

        # img0 in yolov7 is self.img0
        # img  in yolov7 is self.img

        self.img = letterbox(self.img0, self.image_size, stride=self.stride)[0]
        self.img = self.img[:, :, ::-1].transpose(2, 0, 1)
        self.img: np.ndarray = np.ascontiguousarray(self.img)

        # self.img0, self.img
        self.img = torch.from_numpy(self.img).to(self.device)
        self.img = self.img.half() if self.half else self.img.float()  # uint8 to fp16/32
        self.img /= 255.0  # 0 - 255 to 0.0 - 1.0
        if self.img.ndimension() == 3:
            self.img = self.img.unsqueeze(0)

        # Warmup
        if self.device.type != 'cpu' and (old_img_b != self.img.shape[0] or old_img_h != self.img.shape[2] or old_img_w != self.img.shape[3]):
            old_img_b = self.img.shape[0]
            old_img_h = self.img.shape[2]
            old_img_w = self.img.shape[3]
            for i in range(3):
                self.model(self.img, augment=self.augment)[0]
        
        # Inference
        t1 = time_synchronized()
        with torch.no_grad():   # Calculating gradients would cause a GPU memory leak
            pred = model(self.img, augment=self.augment)[0]
        t2 = time_synchronized()

        # Apply NMS
        pred = non_max_suppression(pred, self.conf_thres, self.iou_thres)
        t3 = time_synchronized()

        # 원본 이미지 저장. 자른 이미지에 bounding box 침범 문제 해결 위함
        test_img = copy(self.im0)

        # ** 프레임당 저장할 리스트 정의  ---- START
        # 각 box의 color 히스토그램 리스트에 저장
        redhist_list, bluehist_list, greenhist_list = [], [], []
        # frame당 박스 중앙 좌표 저장한 리스트, frame 당 박스 이미지 저장 리스트
        centercoord_list, frameboximage_list = [], []
        # 박스 색 변화 위한 리스트 
        c_color_list = []
        # ** 프레임당 저장할 리스트 정의  ----  END

        # FIXME have to meet the requirement of Yolov7
        # TODO  check the cord
        # frame의 여러 box 있는 것 loop 돌리기
        # Process detections
        for i, det in enumerate(pred):  # detections per image
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


        
        


class Position:
    def __init__(self, x, y):
        self.x = x
        self.y = y

class Histogram:
    def __init__(self, histogram, center_x):
        self.histogram = histogram
        self.center_x = center_x
    

if __name__ == '__main__':
    with torch.no_grad():
        process = DetectYolo("model_name")
        process.initRos() # subscriber registeration