#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
from math import cos,sin,pi,sqrt,pow
from geometry_msgs.msg import Point32,PoseStamped
from nav_msgs.msg import Odometry,Path

class local_path_pub :
    def __init__(self):
        rospy.init_node('local_path_pub', anonymous=True)
        #TODO: (1) Global Path 와 Odometry 데이터 subscriber 생성 
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/global_path', Path, self.global_path_callback)

        #TODO: (2) Local Path publisher 선언
        self.local_path_pub = rospy.Publisher('/local_path', Path, queue_size=10)
        
        # 초기화
        self.is_odom = False
        self.is_path = False

        #TODO: (3) Local Path 의 Size 결정
        self.local_path_size = 50
        rate = rospy.Rate(20) # 20hz
        while not rospy.is_shutdown():
            if self.is_odom == True and self.is_path == True:
                local_path_msg = Path()
                local_path_msg.header.frame_id = '/map'
                x = self.x
                y = self.y

                #TODO: (5) Global Path 에서 차량 위치와 가장 가까운 포인트(current Waypoint) 탐색
                min_dis = float('inf')
                current_waypoint = -1
                i = -1
                for pose in self.global_path_msg.poses:
                    i += 1
                    dis = sqrt(
                        pow(x - pose.pose.position.x, 2) + pow(y - pose.pose.position.y, 2)
                    )
                    if dis < min_dis:
                        min_dis = dis
                        current_waypoint = i
                
                #TODO: (6) 가장 가까운 포인트(current Waypoint) 위치부터 Local Path 생성 및 예외 처리
                if current_waypoint != -1 :
                    if current_waypoint + self.local_path_size < len(self.global_path_msg.poses):
                        for i in range(current_waypoint, current_waypoint + self.local_path_size):
                            read_pose = PoseStamped()
                            read_pose.pose.position.x = self.global_path_msg.poses[i].pose.position.x
                            read_pose.pose.position.y = self.global_path_msg.poses[i].pose.position.y
                            read_pose.pose.orientation.w = 1
                            local_path_msg.poses.append(read_pose)
                    else :
                        for i in range(current_waypoint, len(self.global_path_msg.poses)):
                            read_pose = PoseStamped()
                            read_pose.pose.position.x = self.global_path_msg.poses[i].pose.position.x
                            read_pose.pose.position.y = self.global_path_msg.poses[i].pose.position.y
                            read_pose.pose.orientation.w = 1
                            local_path_msg.poses.append(read_pose)

                #TODO: (7) Local Path 메세지 Publish
                self.local_path_pub.publish(local_path_msg)

            rate.sleep()

    def odom_callback(self,msg):
        self.is_odom = True
        #TODO: (4) 콜백함수에서 처음 메시지가 들어오면 초기 위치를 저장
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def global_path_callback(self,msg):
        self.is_path = True
        self.global_path_msg = msg

if __name__ == '__main__':
    try:
        test_track=local_path_pub()
    except rospy.ROSInterruptException:
        pass
