#!/usr/bin/env python
# -*- coding: utf-8 -*-
from re import I
import rospy
import rospkg
from math import sqrt, pow
from geometry_msgs.msg import Point32,PoseStamped
from nav_msgs.msg import Odometry

class pathMaker :    
    def __init__(self, pkg_name = 'route', path_name = '/no_parking_lc_1.txt'):
        rospy.init_node('path_maker', anonymous=True)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        # 초기화
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_z = 0.0
        self.is_odom=False

        #TODO: (1) 저장할 경로 및 텍스트파일 이름을 정하고, 쓰기 모드로 열기
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path(pkg_name)
        full_path = pkg_path + '/path' + path_name
        self.f = open(full_path, 'w')
        while not rospy.is_shutdown():
            if self.is_odom == True :
                # Ego 위치 기록
                self.path_make()
        self.f.close()

    def path_make(self):
        x = self.x
        y = self.y
        z = 0.0

        #TODO: (3) 콜백함수에서 이전 위치와 현재 위치의 거리 계산
        distance = sqrt(abs(x - self.prev_x)**2 + abs(y - self.prev_y)**2 + abs(z - self.prev_z)**2)

        #TODO: (4) 이전 위치보다 0.5m 이상일 때 위치를 저장        
        if distance > 0.5:  
            data ='{0}\t{1}\t{2}\n'.format(x,y,z)
            self.f.write(data)
            self.prev_x = x
            self.prev_y = y
            self.prev_z = z
            rospy.loginfo(data)


    def odom_callback(self,msg):
        self.is_odom = True
        #TODO: (2) 콜백함수에서 처음 메시지가 들어오면 초기 위치를 저장
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        self.orientation_x = msg.pose.pose.orientation.x 
        self.orientation_y = msg.pose.pose.orientation.y
        self.orientation_z = msg.pose.pose.orientation.z
        self.orientation_w = msg.pose.pose.orientation.w

if __name__ == '__main__' :
    try:
        p_m=pathMaker()
    except rospy.ROSInternalException:
        pass
            