#!/usr/bin/env python
# -*- coding: utf-8 -*-

from math import atan2, degrees, sqrt
import os
import rospy
import cv2
import numpy as np
import boto3
import mysql.connector
from mysql.connector import Error
from datetime import datetime

from sensor_msgs.msg import CompressedImage
from morai_msgs.msg import ObjectStatusList, EgoVehicleStatus
from cv_bridge import CvBridgeError

class NPCDetector:
    def __init__(self):
        rospy.Subscriber("/image_jpeg/compressed2", CompressedImage, self.callback2)
        rospy.Subscriber("/Ego_topic" , EgoVehicleStatus , self.EgoStatus_callback)
        rospy.Subscriber("/Object_topic", ObjectStatusList , self.Object_callback)
        self.rate = rospy.Rate(20)
        self.home_dir = os.environ.get('HOME')

        self.ego_pos = {
            'x': 0,
            'y': 0,
        }
        self.car_info = {
            'name': '',
            'position': {
                'x': 0,
                'y': 0
            },
            'velocity': {
                'x': 0,
                'y': 0,
            }
        }
        self.car_capture_flag = 0

        self.bucket_name = 'gotothemars'    # 버킷 이름
        self.object_path = ''               # S3에 올릴 파일 경로
        self.file_path = ''                 # 로컬 파일의 경로

        self.connection = mysql.connector.connect(
            host = 'j7c109.p.ssafy.io',
            database = 'NextDB',
            user = 'gotothemars',
            password = 'c109_mars'
        )
        self.cur = self.connection.cursor()
        self.s3 = boto3.client('s3')

    def isExistData(self):
        try:
            sql_fetch_blob_query = """
                SELECT * from tb_car_illegal where car_num = %s
            """
            insert_tuple = (self.car_info['name'],)
            self.cur.execute(sql_fetch_blob_query, insert_tuple)
            record = self.cur.fetchall()
            for _ in record:
                return True
            return False

        except Error as error:
            print(error)

    def sendDataToDB(self):
        try:
            current_time = datetime.now().replace(microsecond=0)
            cnt = 1
            if self.isExistData():
                tmp_query = """
                    select (prior_cnt) from tb_car_illegal where car_num = %s
                """
                tmp_tuple = (self.car_info['name'],)
                self.cur.execute(tmp_query, tmp_tuple)
                for (prior_cnt,) in self.cur:
                    cnt = prior_cnt + 1
                    break

                sql_insert_blob_query = """
                    update tb_car_illegal set time = %s, npc_x = %s, npc_y = %s, prior_cnt = %s where car_num = %s
                """
                insert_tuple = (
                    current_time, self.car_info['position']['x'],
                    self.car_info['position']['y'], cnt,
                    self.car_info['name']
                )
            else:
                sql_insert_blob_query = """
                    insert into tb_car_illegal (car_num, time, npc_x, npc_y, prior_cnt, image_data) values (%s, %s, %s, %s, %s, %s)
                """
                img_data = "https://gotothemars.s3.ap-northeast-2.amazonaws.com/car_picture/{}.jpg".format(self.car_info['name'])
                insert_tuple = (
                    self.car_info['name'], current_time,
                    self.car_info['position']['x'], self.car_info['position']['y'],
                    cnt, img_data
                )

            self.cur.execute(sql_insert_blob_query, insert_tuple)
            self.connection.commit()

        except Error as error:
            print(error)


    def callback2(self, msg):
        self.rate.sleep()
        try:
            np_arr = np.fromstring(msg.data, np.uint8)
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)

        # cv2.imshow("Image window", img_bgr)
        # cv2.waitKey(1) 

        # 나와 상대 간 거리, 각도, 신호 여부, 속도, 차선 물었는가?
        pos_x = self.ego_pos['x'] - self.car_info['position']['x']
        pos_y = self.ego_pos['y'] - self.car_info['position']['y']
        dist = sqrt(pos_x**2 + pos_y**2)
        angle = degrees(atan2(pos_y, pos_x))
        print(dist, angle)

        if self.car_info['name'] != '' and self.car_capture_flag == 0 \
        and 0 <= abs(self.car_info['velocity']['x']) <= 1 \
        and 0 <= abs(self.car_info['velocity']['y']) <= 1 \
        and 5 <= dist <= 6 \
        and -180 <= angle <= -120:
            print("진입 완료")
            self.file_path = self.home_dir + '/S07P22C109/car_capture/{}.jpg'.format(self.car_info['name'])
            self.object_path = 'car_picture/{}.jpg'.format(self.car_info['name'])
            cv2.imwrite(self.file_path, img_bgr)
            self.s3.upload_file(self.file_path, self.bucket_name, self.object_path)
            self.sendDataToDB()

            self.car_info = {
                'name': '',
                'position': {
                    'x': 0,
                    'y': 0
                },
                'velocity': {
                    'x': 0,
                    'y': 0,
                }
            }
            self.object_path = ''
            self.file_path = ''

        if self.car_capture_flag == 3:
            self.car_capture_flag = 0
        else:
            self.car_capture_flag += 1

    def Object_callback(self, data):
        for _ in range(data.num_of_npcs) :
            self.car_info = {
                'name': data.npc_list[0].name,
                'position': {
                    'x': data.npc_list[0].position.x,
                    'y': data.npc_list[0].position.y
                },
                'velocity': {
                    'x': data.npc_list[0].velocity.x,
                    'y': data.npc_list[0].velocity.y
                }
            }
            break

    def EgoStatus_callback(self, data):
        self.ego_pos = {
            'x': data.position.x,
            'y': data.position.y
        }

if __name__ == '__main__':
    try:
        rospy.init_node('npc_detector', anonymous=True)
        npc_detector = NPCDetector()
        rospy.spin() 
    finally:
        print("MySQL closed")
        npc_detector.cur.close()
        npc_detector.connection.close()