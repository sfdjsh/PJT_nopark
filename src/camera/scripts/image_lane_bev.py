#!/usr/bin/env python
# -*- coding: utf-8 -*-
 
import rospy
import cv2
import numpy as np
import os, rospkg
import json

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError

def warp_image(img, source_prop):
    image_size = (img.shape[1], img.shape[0])
    x = img.shape[1]
    y = img.shape[0]
    
    #TODO: (2) 원본 이미지 내 Warping 영역과 매칭 될 목적지 지점 정의
    destination_points = np.float32([[0, y], [0, 0], [x, 0], [x, y]])
    source_points = source_prop * np.float32([[x, y]]* 4)
    
    #TODO: (3) 원근 맵 행렬 생성 함수를 이용하여 매핑 좌표에 대한 원근 맵 행렬을 생성합니다.
    perspective_transform = cv2.getPerspectiveTransform(source_points, destination_points)

    #TODO: (4) 이후 원근 맵 행렬에 대한 기하학적 변환을 진행합니다.
    warped_img = cv2.warpPerspective(img, perspective_transform, image_size)

    return warped_img


class IMGParser:
    def __init__(self):

        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        self.img_bgr = None

        #TODO: (1) 이미지 warping 영역 지정
        self.source_prop = np.float32([[0, 0.7291], [0.4375, 0.4167], [0.5625, 0.4167], [1, 0.7291]])

    def callback(self, msg):
        try:
            np_arr = np.fromstring(msg.data, np.uint8)
            self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)

        #TODO: (2-0) 이미지 Warping 시작
        img_warp = warp_image(self.img_bgr, self.source_prop)
        img_concat = np.concatenate([self.img_bgr, img_warp], axis=1)
        cv2.imshow("Image window", img_concat)
        cv2.waitKey(1) 


def main():
    rospy.init_node('lane_birdview', anonymous=True)
    image_parser = IMGParser()
    rospy.spin()

if __name__ == '__main__':
    main()
