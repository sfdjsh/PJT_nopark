#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import rospy

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped,Point32
from sensor_msgs.msg import PointCloud

current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)

from lib.mgeo.class_defs import *

class get_mgeo :
    def __init__(self):
        rospy.init_node('test', anonymous=True)
        self.link_pub = rospy.Publisher('link',PointCloud, queue_size=10)
        self.node_pub = rospy.Publisher('node',PointCloud, queue_size=10)

        #TODO: (1) Mgeo data 읽어온 후 데이터 확인
        load_path = os.path.normpath(os.path.join(current_path, 'lib/mgeo_data/R_KR_PG_K-City'))
        mgeo_planner_map = MGeo.create_instance_from_json(load_path)

        node_set = mgeo_planner_map.node_set
        link_set = mgeo_planner_map.link_set

        self.nodes=node_set.nodes
        self.links=link_set.lines

        self.link_msg=self.getAllLinks()
        self.node_msg=self.getAllNode()

        rate = rospy.Rate(1) 
        while not rospy.is_shutdown():
            #TODO: (4) 변환한 Link, Node 정보 Publish
            self.link_pub.publish(self.link_msg)
            self.node_pub.publish(self.node_msg)
            rate.sleep()


    def getAllLinks(self):
        all_link=PointCloud()
        all_link.header.frame_id='map'
        #TODO: (2) Link 정보 Point Cloud 데이터로 변환
        for link_idx in self.links :
            for point in self.links[link_idx].points:
                temp = Point32()
                temp.x = point[0]
                temp.y = point[1]
                temp.z = point[2]
                all_link.points.append(temp)
        return all_link
    
    def getAllNode(self):
        all_node=PointCloud()
        all_node.header.frame_id='map'

        #TODO: (3) Node 정보 Point Cloud 데이터로 변환
        for node_idx in self.nodes:
            point = self.nodes[node_idx].point
            temp = Point32()
            temp.x = point[0]
            temp.y = point[1]
            temp.z = point[2]
            all_node.points.append(temp)
        return all_node

if __name__ == '__main__':
    test_track=get_mgeo()