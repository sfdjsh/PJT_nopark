#!/usr/bin/env python
# -*- coding: utf-8 -*-
 
import rospy
import cv2
import numpy as np
import os, rospkg

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError

# image_lane_roi 는 카메라 센서를 통하여 받아온 이미지에 관심있는 부분만(차선) 만 남기고
# 나머지 부분은 마스킹 하는 이미리 처리입니다. 관심 영역을 지정하고, 마스크를 생성, 마스크를 이미지에 합치는 과정을
# 합니다. 

class IMGParser:
    def __init__(self):
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        x = 640
        y = 480
        self.crop_pts = np.array([x, y])

    def callback(self, msg):
        try:
            np_arr = np.fromstring(msg.data, np.uint8)
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)

        mask_color = self.mask_roi(img_bgr)
        # mask_color = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        mask_hsv = cv2.cvtColor(mask_color, cv2.COLOR_BGR2HSV)

        lower_green = np.array([50, 50, 80])
        upper_green = np.array([90, 255, 255])
        lower_red = np.array([-10, 30, 50])
        upper_red = np.array([10, 255, 255])
        lower_yellow = np.array([11, 50, 50])
        upper_yellow = np.array([30, 200, 200])

        img_green = cv2.inRange(mask_hsv, lower_green, upper_green)
        img_gresult = cv2.bitwise_and(mask_color, mask_color, mask=img_green)
        img_red = cv2.inRange(mask_hsv, lower_red, upper_red)
        img_rresult = cv2.bitwise_and(mask_color, mask_color, mask=img_red)
        img_yellow = cv2.inRange(mask_hsv, lower_yellow, upper_yellow)
        img_yresult = cv2.bitwise_and(mask_color, mask_color, mask=img_yellow)

        img_light = cv2.bitwise_or(img_gresult, img_rresult, img_yresult)
        img_concat = np.concatenate([mask_hsv, img_light], axis=1)

        cv2.imshow("Image window", img_concat)
        cv2.waitKey(1)
        # cv2.imshow("Image window", )
        # cv2.waitKey(1) 

        # if len(self.mask.shape)==3:
        #     img_concat = np.concatenate([img_bgr, self.mask], axis=1)
        # else:
        #     img_concat = np.concatenate([img_bgr, self.mask_color], axis=1)

    def mask_roi(self, img):
        h = img.shape[0]
        w = img.shape[1]
        
        if len(img.shape)==3:
            c = img.shape[2]
            mask = np.zeros((h, w, c), dtype=np.uint8)
            mask_value = (255, 255, 255)

        else:
            mask = np.zeros((h, w), dtype=np.uint8)
            mask_value = (255)
        
        points = np.array([[0, 0], [200, 150], [440, 150], [640, 0]])
        cv2.fillPoly(mask, [points], mask_value, 1)

        mask = cv2.bitwise_and(img, mask)
        return mask


if __name__ == '__main__':
    rospy.init_node('image_parser', anonymous=True)
    image_parser = IMGParser()
    rospy.spin() 
