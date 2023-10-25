#! /usr/bin/python3
import rospy
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import torch
from numpy import random

from models.experimental import attempt_load
from utils.torch_utils import select_device, load_classifier, time_synchronized, TracedModel

class DetectYolo:
    def __init__(self, weights):
        self.bridge = CvBridge()

        # initialize
        self.device = select_device('')
        self.half = self.device.type != 'cpu'

        # load model
        self.model = attempt_load(weights, map_location=self.device)
        self.stride = int(self.model.stride.max())

        if self.half:
            self.model.half()

        # Get names and colors
        self.names = self.model.module.names if hasattr(self.model, 'module') else self.model.names
        self.colors = [[random.randint(0, 255) for _ in range(3)] for _ in self.names]

        # 여기에 있는 배열의 각 저장된 하나씩은 frame의 정보다. 
        # 동영상 용 frame 별 각 box 당 히스토그램 저장 위한 list
        self.redhis, self.bluehis, self.greenhis = [], [], []

        # frame 당 중앙박스 좌표 저장 리스트, frame 당 박스 이미지 저장 리스트
        self.center_coord, self.frame_box_img = [], []

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
            self.image = self.bridge.imgmsg_to_cv2(image_data, image_data.encoding)
        except CvBridgeError as e:
            rospy.INFO("Error convert image %s", e) 
            return

        self.image = torch.from_numpy(self.image).to(self.device)
        self.image = self.image.half() if half else self.image.float()
        self.image /= 255.0 # 0 ~ 255 to 0.0 ~ 1.0
        if self.image.ndimension() == 3:
            self.image = self.image.unsqueeze(0)

        # TODO making yolov7 process here



        
        


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
        process = DetectYolo()
        process.initRos()