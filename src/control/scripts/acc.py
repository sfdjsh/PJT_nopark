#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
from math import cos,sin,pi,sqrt,pow,atan2
from geometry_msgs.msg import Point,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry,Path
from morai_msgs.msg import CtrlCmd,EgoVehicleStatus,ObjectStatusList
import numpy as np
import sympy as sy
import tf
from tf.transformations import euler_from_quaternion,quaternion_from_euler

class pure_pursuit :
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)

        #TODO: (1) subscriber, publisher 선언
        rospy.Subscriber("/global_path", Path, self.global_path_callback)
        rospy.Subscriber("/local_path", Path, self.path_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.status_callback)
        rospy.Subscriber("/Object_topic", ObjectStatusList, self.object_info_callback)
        self.ctrl_cmd_pub = rospy.Publisher("/ctrl_cmd", CtrlCmd, queue_size=10)

        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType = 1

        self.is_path = False
        self.is_odom = False
        self.is_status = False
        self.is_global_path = False

        self.is_look_forward_point = False

        self.forward_point = Point()
        self.current_postion = Point()

        self.vehicle_length = 4.3561
        self.target_velocity = 50
        self.min_lfd = 5
        self.max_lfd = 20
        self.lfd_gain = 0.9

        self.pid = pidControl()
        self.adaptive_cruise_control = AdaptiveCruiseControl(
            velocity_gain = 0.5, distance_gain = 1,
            time_gap = 0.8, vehicle_length = 4.3561
        )
        self.vel_planning = velocityPlanning(self.target_velocity/3.6, 0.15)

        while True:
            if self.is_global_path == True:
                self.velocity_list = self.vel_planning.curvedBaseVelocity(self.global_path, 50)
                break
            else:
                rospy.loginfo('Waiting global path data')

        rate = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown():
            if self.is_path == True and self.is_odom == True and self.is_status == True:
                self.lfd = self.lfd_gain * self.status_msg.velocity.x
                if self.lfd >= self.max_lfd: self.lfd = self.max_lfd
                elif self.lfd <= self.min_lfd: self.lfd = self.min_lfd

                # global_obj,local_obj
                result = self.calc_vaild_obj([self.current_postion.x,self.current_postion.y,self.vehicle_yaw],self.object_data)
                
                global_npc_info = result[0] 
                local_npc_info = result[1] 
                global_ped_info = result[2] 
                local_ped_info = result[3] 
                global_obs_info = result[4] 
                local_obs_info = result[5] 
                
                self.current_waypoint = self.get_current_waypoint([self.current_postion.x,self.current_postion.y],self.global_path)
                self.target_velocity = self.velocity_list[self.current_waypoint]*3.6

                steering = self.calc_pure_pursuit()
                if self.is_look_forward_point :
                    self.ctrl_cmd_msg.steering = steering
                else : 
                    rospy.loginfo("no found forward point")
                    self.ctrl_cmd_msg.accel = 0.0
                    self.ctrl_cmd_msg.brake = 1.0
                    self.ctrl_cmd_msg.steering=0.0

                self.adaptive_cruise_control.check_object(self.path ,global_npc_info, local_npc_info
                                                                    ,global_ped_info, local_ped_info
                                                                    ,global_obs_info, local_obs_info)
                self.target_velocity = self.adaptive_cruise_control.get_target_velocity(local_npc_info, local_ped_info, local_obs_info,
                                                                                                        self.status_msg.velocity.x, self.target_velocity/3.6)

                output = self.pid.pid(self.target_velocity,self.status_msg.velocity.x*3.6)

                if output > 0.0:
                    self.ctrl_cmd_msg.accel = output
                    self.ctrl_cmd_msg.brake = 0.0
                else:
                    self.ctrl_cmd_msg.accel = 0.0
                    self.ctrl_cmd_msg.brake = -output

                #TODO: (10) 제어입력 메세지 Publish
                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

            rate.sleep()

    def path_callback(self,msg):
        self.is_path=True
        self.path=msg  

    def odom_callback(self,msg):
        self.is_odom=True
        odom_quaternion=(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        _,_,self.vehicle_yaw=euler_from_quaternion(odom_quaternion)
        self.current_postion.x=msg.pose.pose.position.x
        self.current_postion.y=msg.pose.pose.position.y

    def status_callback(self,msg): ## Vehicl Status Subscriber 
        self.is_status=True
        self.status_msg=msg    
        
    def global_path_callback(self,msg):
        self.global_path = msg
        self.is_global_path = True

    def object_info_callback(self,data): ## Object information Subscriber
        self.is_object_info = True
        self.object_data = data 

    def get_current_waypoint(self,ego_status,global_path):
        min_dist = float('inf')        
        currnet_waypoint = -1     

        ego_pose_x = ego_status[0]
        ego_pose_y = ego_status[1]

        for i,pose in enumerate(global_path.poses):
            dx = ego_pose_x - pose.pose.position.x
            dy = ego_pose_y - pose.pose.position.y

            dist = sqrt(pow(dx,2)+pow(dy,2))
            if min_dist > dist :
                min_dist = dist
                currnet_waypoint = i
        return currnet_waypoint

    def calc_vaild_obj(self,status_msg,object_data):
        
        self.all_object = object_data        
        ego_pose_x = status_msg[0]
        ego_pose_y = status_msg[1]
        ego_heading = status_msg[2]
        
        global_npc_info = []
        local_npc_info  = []
        global_ped_info = []
        local_ped_info  = []
        global_obs_info = []
        local_obs_info  = []

        num_of_object = self.all_object.num_of_npcs + self.all_object.num_of_obstacle + self.all_object.num_of_pedestrian        
        if num_of_object > 0:

            #translation
            tmp_theta=ego_heading
            tmp_translation=[ego_pose_x, ego_pose_y]
            tmp_t=np.array([[cos(tmp_theta), -sin(tmp_theta), tmp_translation[0]],
                            [sin(tmp_theta),  cos(tmp_theta), tmp_translation[1]],
                            [0             ,               0,                  1]])
            tmp_det_t=np.array([[tmp_t[0][0], tmp_t[1][0], -(tmp_t[0][0] * tmp_translation[0] + tmp_t[1][0]*tmp_translation[1])],
                                [tmp_t[0][1], tmp_t[1][1], -(tmp_t[0][1] * tmp_translation[0] + tmp_t[1][1]*tmp_translation[1])],
                                [0,0,1]])

            #npc vehicle translation        
            for npc_list in self.all_object.npc_list:
                global_result=np.array([[npc_list.position.x],[npc_list.position.y],[1]])
                local_result=tmp_det_t.dot(global_result)
                if local_result[0][0]> 0 :        
                    global_npc_info.append([npc_list.type,npc_list.position.x,npc_list.position.y,npc_list.velocity.x])
                    local_npc_info.append([npc_list.type,local_result[0][0],local_result[1][0],npc_list.velocity.x])

            #ped translation
            for ped_list in self.all_object.pedestrian_list:
                global_result=np.array([[ped_list.position.x],[ped_list.position.y],[1]])
                local_result=tmp_det_t.dot(global_result)
                if local_result[0][0]> 0 :
                    global_ped_info.append([ped_list.type,ped_list.position.x,ped_list.position.y,ped_list.velocity.x])
                    local_ped_info.append([ped_list.type,local_result[0][0],local_result[1][0],ped_list.velocity.x])

            #obs translation
            for obs_list in self.all_object.obstacle_list:
                global_result=np.array([[obs_list.position.x],[obs_list.position.y],[1]])
                local_result=tmp_det_t.dot(global_result)
                if local_result[0][0]> 0 :
                    global_obs_info.append([obs_list.type,obs_list.position.x,obs_list.position.y,obs_list.velocity.x])
                    local_obs_info.append([obs_list.type,local_result[0][0],local_result[1][0],obs_list.velocity.x])
                
        return global_npc_info, local_npc_info, global_ped_info, local_ped_info, global_obs_info, local_obs_info

    def calc_pure_pursuit(self):                
        vehicle_position=self.current_postion
        self.is_look_forward_point= False
        trans_pos = [vehicle_position.x, vehicle_position.y]

        # TODO: (2) 좌표 변환 행렬 생성
        trans_matrix = np.array([
            [cos(-self.vehicle_yaw), sin(-self.vehicle_yaw), 0],
            [-sin(-self.vehicle_yaw), cos(-self.vehicle_yaw), 0],
            [0, 0, 1]
        ])

        det_trans_matrix = np.linalg.inv(trans_matrix)

        for num, i in enumerate(self.path.poses):
            path_point = [i.pose.position.x, i.pose.position.y]
            global_path_point = [path_point[0] - trans_pos[0], path_point[1] - trans_pos[1], 1]
            local_path_point = det_trans_matrix.dot(global_path_point)

            if local_path_point[0] > 0:
                dis = sqrt(pow(local_path_point[0], 2) + pow(local_path_point[1], 2))
                if dis >= self.lfd:
                    self.forward_point.x = local_path_point[0]
                    self.forward_point.y = local_path_point[1]
                    self.forward_point.z = local_path_point[2]

                    self.is_look_forward_point = True
                    break

        #TODO: (3) Steering 각도 계산
        theta = atan2(self.forward_point.y, self.forward_point.x)
        steering = atan2(
            2 * self.vehicle_length * sin(theta),
            self.lfd
        )

        return steering

class pidControl:
    def __init__(self):
        self.p_gain = 0.4
        self.i_gain = 0.0
        self.d_gain = 0.03
        self.prev_error = 0
        self.i_control = 0
        self.controlTime = 0.02

    def pid(self, target_vel, current_vel):
        error = target_vel - current_vel

        #TODO: (4) PID 제어 생성
        p_control = self.p_gain * error
        d_control = self.d_gain * (error - self.prev_error) / self.controlTime
        output = p_control + d_control
        self.prev_error = error
        return output

class velocityPlanning:
    def __init__ (self,car_max_speed, road_friciton):
        self.car_max_speed = car_max_speed
        self.road_friction = road_friciton

    def curvedBaseVelocity(self, gloabl_path, point_num):
        out_vel_plan = []

        for i in range(0,point_num):
            out_vel_plan.append(self.car_max_speed)

        for i in range(point_num, len(gloabl_path.poses) - point_num):
            x_list = []
            y_list = []
            for box in range(-point_num, point_num):
                x = gloabl_path.poses[i + box].pose.position.x
                y = gloabl_path.poses[i + box].pose.position.y
                x_list.append([-2*x, -2*y ,1])
                y_list.append((-x*x) - (y*y))

            #TODO: (5) 도로의 곡률 계산
            x_trans = np.transpose(x_list)
            target_list = np.linalg.inv(x_trans.dot(x_list)).dot(x_trans).dot(y_list)
            r = sqrt(target_list[0]**2 + target_list[1]**2 - target_list[2])

            #TODO: (6) 곡률 기반 속도 계획
            v_max = sqrt(r * 9.8 * self.road_friction)

            if v_max > self.car_max_speed:
                v_max = self.car_max_speed

            out_vel_plan.append(v_max)
        for i in range(len(gloabl_path.poses) - point_num, len(gloabl_path.poses)-10):
            out_vel_plan.append(30)

        for i in range(len(gloabl_path.poses) - 10, len(gloabl_path.poses)):
            out_vel_plan.append(0)

        return out_vel_plan

class AdaptiveCruiseControl:
    def __init__(self, velocity_gain, distance_gain, time_gap, vehicle_length):
        self.npc_vehicle=[False,0]
        self.object=[False,0]
        self.Person=[False,0]
        self.velocity_gain = velocity_gain
        self.distance_gain = distance_gain
        self.time_gap = time_gap
        self.vehicle_length = vehicle_length

        self.object_type = None
        self.object_distance = 0
        self.object_velocity = 0

    def check_object(self,ref_path, global_npc_info, local_npc_info, 
                                    global_ped_info, local_ped_info, 
                                    global_obs_info, local_obs_info):

        #TODO: (8) 경로상의 장애물 유무 확인 (차량, 사람, 정지선 신호)

        # 주행 경로 상 보행자 유무 파악
        min_rel_distance=float('inf')
        if len(global_ped_info) > 0 :        
            for i in range(len(global_ped_info)):
                for path in ref_path.poses :
                    if global_ped_info[i][0] == 0 : # type=0 [pedestrian]
                        dis = sqrt(pow(global_ped_info[i][1] - path.pose.position.x, 2) + pow(global_ped_info[i][2]-path.pose.position.y, 2))
                        if dis<2.35:
                            rel_distance = sqrt(pow(local_ped_info[i][1], 2) + pow(local_ped_info[i][2], 2))       
                            if rel_distance < min_rel_distance:
                                min_rel_distance = rel_distance
                                self.Person=[True,i]

        # 주행 경로 상 NPC 차량 유무 파악
        if len(global_npc_info) > 0 :            
            for i in range(len(global_npc_info)):
                for path in ref_path.poses :      
                    if global_npc_info[i][0] == 1 : # type=1 [npc_vehicle] 
                        dis = sqrt(pow(global_npc_info[i][1] - path.pose.position.x, 2) + pow(global_npc_info[i][2]-path.pose.position.y, 2))  
                        if dis < 5:
                            # npc 차량 각도
                            rel_distance = sqrt(pow(local_npc_info[i][1], 2) + pow(local_npc_info[i][2], 2))   
                            if rel_distance < min_rel_distance:
                                min_rel_distance = rel_distance
                                self.npc_vehicle=[True,i]

                                
        # 주행 경로 상 Obstacle 유무 파악
        if len(global_obs_info) > 0 :            
            for i in range(len(global_obs_info)):
                for path in ref_path.poses :      
                    if global_obs_info[i][0] == 2 : # type=1 [obstacle] 
                        dis = sqrt(pow(global_obs_info[i][1] - path.pose.position.x, 2) + pow(global_obs_info[i][2]-path.pose.position.y, 2))
                        # 각도 추가
                        if dis<2.35:
                            rel_distance = sqrt(pow(local_obs_info[i][1], 2) + pow(local_obs_info[i][2], 2))   
                            if rel_distance < min_rel_distance:
                                min_rel_distance = rel_distance
                                self.object=[True, i] 
        
    def get_target_velocity(self, local_npc_info, local_ped_info, local_obs_info, ego_vel, target_vel): 
        #TODO: (9) 장애물과의 속도와 거리 차이를 이용하여 ACC 를 진행 목표 속도를 설정
        out_vel =  target_vel
        default_space = 8
        time_gap = self.time_gap
        v_gain = self.velocity_gain
        x_errgain = self.distance_gain

        if self.npc_vehicle[0] and len(local_npc_info) != 0: #ACC ON_vehicle          
            front_vehicle = [local_npc_info[self.npc_vehicle[1]][1], local_npc_info[self.npc_vehicle[1]][2], local_npc_info[self.npc_vehicle[1]][3]]
            
            dis_safe = ego_vel * time_gap + default_space
            dis_rel = sqrt(pow(front_vehicle[0],2) + pow(front_vehicle[1],2))            
            vel_rel=((front_vehicle[2] / 3.6) - ego_vel)                        
            acceleration = vel_rel * v_gain - x_errgain * (dis_safe - dis_rel)

            out_vel = ego_vel + acceleration      

        if self.Person[0] and len(local_ped_info) != 0: #ACC ON_Pedestrian
            Pedestrian = [local_ped_info[self.Person[1]][1], local_ped_info[self.Person[1]][2], local_ped_info[self.Person[1]][3]]
            
            dis_safe = ego_vel* time_gap + default_space
            dis_rel = sqrt(pow(Pedestrian[0],2) + pow(Pedestrian[1],2))            
            vel_rel = (Pedestrian[2] - ego_vel)              
            acceleration = vel_rel * v_gain - x_errgain * (dis_safe - dis_rel)    

            out_vel = ego_vel + acceleration
   
        if self.object[0] and len(local_obs_info) != 0: #ACC ON_obstacle                      
            Obstacle = [local_obs_info[self.object[1]][1], local_obs_info[self.object[1]][2], local_obs_info[self.object[1]][3]]
            
            dis_safe = ego_vel* time_gap + default_space
            dis_rel = sqrt(pow(Obstacle[0],2) + pow(Obstacle[1],2))            
            vel_rel = (Obstacle[2] - ego_vel)
            acceleration = vel_rel * v_gain - x_errgain * (dis_safe - dis_rel)    

            out_vel = ego_vel + acceleration           

        return out_vel * 3.6


if __name__ == '__main__':
    try:
        test_track=pure_pursuit()
    except rospy.ROSInterruptException:
        pass