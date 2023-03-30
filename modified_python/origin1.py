from operator import truediv
from airsim.types import ImageRequest, Quaternionr, Vector3r
import airsim
import time 
import numpy as np
import os #操作文件和目录
import pprint #打印出任何python数据结构类和方法
import cv2
from scipy.spatial.transform import Rotation as R  #三维旋转
import math
import circle_finder

class uav_setpoints: #储存地图环境中已知的参数
    def __init__(self) -> None: #初始化
        self.circle_setpoint_moveToPositionAsync = (   #预设圈的位置和运行速度（x,y,z,v), 位置是大致坐标！
            (0.0, 0.0, -2, 15), #加入起点，便于计算
            (15.5, -19.6 , -3.5, 1),
            (22, -41.2 , -2.5, 5),
            (21, -61.5 , -2, 3),
            (10, -78.2, -2, 3),
            (-9.3, -93, -2.5, 3),
            (-27, -98, -4, 3),
            (-50.1, -103, -5.7, 3)
        )

        self.circle_halfwaypoint_moveToPositionAsync = (
            (7.75, -9.7, -2.75, 15), # 7.75, -9.8, -2.75
            (19.2, -35, -3, 10), # 18.75, -30.4, -3.0
            (22.2, -55, -2.6, 10),
            (15, -76, -1, 7),
            (0.35, -85.6, -1.5, 10),
            (-18.15, -95.5, -3.25, 10),
            (-38.55, -100.5, -4.85, 10)
        )

        self.circle_yaw_rotateToYawAsync = ( #预设圆圈绕 z 轴旋转的度数
            -90,
            -90,
            -100,
            -120,
            -170,
            -170,
            -180
        )

        self.land_setpoint_moveToPositionAsync = (-62.9, -102.3, -4, 3) #降落点的位置和速度

    #数据获取函数
    def get_circle_setpoint(self, id):
        return self.circle_setpoint_moveToPositionAsync[id]
    
    def get_circle_halfwaypoint(self, id):
        return self.circle_halfwaypoint_moveToPositionAsync[id]

    def get_circle_yaw(self, id_from_one):
        return self.circle_yaw_rotateToYawAsync[id_from_one - 1]

    def get_land_setpoint(self):
        return self.land_setpoint_moveToPositionAsync

# ===========================

class airsim_client:
    def __init__(self, ip_addr='127.0.0.1') -> None: #id用于连接场景
        # print("Try to connect {}...".format(ip_addr))
        self.client = airsim.MultirotorClient(ip_addr) #创建无人机
        self.client.confirmConnection()
        self.client.enableApiControl(True)

        # self.dhc = dh.drone_func_class()
        #让circle.finder()初始化，第一个是文件名，第二个是class名
        self.circle_finder = circle_finder.circle_finder(self.client) 
        self.setpoints = uav_setpoints()

    def task_takeoff(self): #让起飞时间尽量短
        self.client.armDisarm(True)
        self.client.takeoffAsync(0.0001).join() #.join()是为了让飞机先起飞然后才进行下一个动作

    def task_cross_circle(self, circle_id): #输入圈的序号
        # print(self.client.getMultirotorState().kinematics_estimated.position) 打印当前坐标位置

        Yawtask=airsim.YawMode()
        Yawtask.is_rate=False

        # halfway_xyz = tuple((self.setpoints.get_circle_setpoint(circle_id - 1)[i] + self.setpoints.get_circle_setpoint(circle_id)[i]) / 2 for i in range(0, 3))
        # print("halfway_xyz : ", halfway_xyz)

        self.client.moveToPositionAsync(*self.setpoints.get_circle_halfwaypoint(circle_id - 1)).join()

        # self.client.moveToPositionAsync(15.5/2,-19.6/2,-5/2, 15).join()
        self.client.rotateToYawAsync(self.setpoints.get_circle_yaw(circle_id)).join()#旋转yaw角，正对障碍物识别
        # self.client.moveToPositionAsync(*self.setpoints.get_circle_setpoint(circle_id)).join() #飞往圈大致的坐标位置
        self.client.hoverAsync().join() # 悬停函数
        # time.sleep(3) #
        # airsim.wait_key('Press any key to rotate')

        # time.sleep(3) #
        circle_xyz = self.circle_finder.get_circle_position_in_wc() #得到圆心的世界坐标
        self.client.moveToPositionAsync(*circle_xyz, 2.5).join()
        self.client.hoverAsync().join()
        # self.dhc.cross_circle(self.client)

        #self.client.rotateToYawAsync(0).join()
        # time.sleep(3)

    # def task_cross_circle(self, circle_id_from_one): #输入圈的序号
    #     self.client.moveToPositionAsync(*self.setpoints.get_circle_setpoint(circle_id_from_one)).join() #飞往圈大致的坐标位置
    #     self.client.hoverAsync().join() # 悬停函数
    #     time.sleep(3) #
    #     #airsim.wait_key('Press any key to rotate')
    #     self.client.rotateToYawAsync(self.setpoints.get_circle_yaw(circle_id_from_one)).join() # 旋转yaw角，正对障碍物识别
    #     # time.sleep(3) #
    #     circle_xyz = self.circle_finder.get_circle_position_in_wc() #得到圆心的世界坐标
    #     self.client.moveToPositionAsync(*circle_xyz, 1).join()
    #     self.client.hoverAsync().join()
    #     # self.dhc.cross_circle(self.client)

    #     self.client.rotateToYawAsync(0).join()
    #     # time.sleep(3)
    
    def task_land(self):
        self.client.moveToPositionAsync(*self.setpoints.get_land_setpoint()).join()
        self.client.hoverAsync().join() # 悬停函数
        time.sleep(3) #
        self.client.landAsync().join()
        self.client.armDisarm(False)

    def begin_task(self):
        # airsim.wait_key('Press any key to takeoff.')
        # print("=========================")
        # print("Taking off...")
        self.task_takeoff()

        for circle_id in range(1, 7 + 1):
            # time.sleep(3)
            print("=========================")
            print("Now try to pass circle {}...".format(circle_id))
            self.task_cross_circle(circle_id)
        
        print("=========================")
        print("Landing...")
        self.task_land()

        print("=========================")
        print("Task is finished.")

