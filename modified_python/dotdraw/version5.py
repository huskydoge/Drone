# 尝试过圈4：高度已经合适
from airsim.types import ImageRequest, Quaternionr, Vector3r
import airsim
import time
import numpy as np
import os
import pprint
import cv2
from scipy.spatial.transform import Rotation as R 
import math
import circle_finder

class uav_setpoints:
    def __init__(self) -> None:
        self.circle_setpoint_moveToPositionAsync = (
            (15.5, -19.6 , -3.5, 5),
            (22, -41.2 , -2.2, 5),
            (21, -61.5 , -2, 3),
            (10.5, -79.2, -0.7, 3), #10, -78.2, -1, 3
            (-8.5, -92.5, -0.7, 3),
            (-27, -98, -4, 3),
            (-49.5, -103, -4.5, 3)
        )

        self.circle_foundpoint_moveToPositionAsync = (
            (16.50, -24.23, -3.62, 3),
            (23.07, -45.60, -3.09, 3),
            (20, -66.04, -1.3, 3), #19.44, -66.04, -2.23, 3
            (3, -100, -0.4, 3),  #7.30, -82.97, -0.74, 3
            (-12.96, -92.56, -0.67, 3),
            (-31.57, -99.23, -3.41, 3),
            (-53.67, -101.75, -3.64, 3)
        )

        self.circle_targetpoint_moveToPositionAsync = (
            (16.50, -24.23, -3.62, 3),
            (23.07, -45.60, -3.09, 3),
            (19, -70, -1.3, 3), #19.44, -66.04, -2.23, 3
            (6.5, -85, -0.4, 3),  #7.30, -82.97, -0.74, 3
            (-12.96, -92.56, -0.67, 3),
            (-31.57, -99.23, -3.41, 3),
            (-53.67, -102.5, -4.0, 3)
        )

        self.point_moveToPositionAsync = (
            (4.70, -3, -2.7, 15),
            (8, -6.62, -2.6, 15),
            (10, -11.00, -2.5, 13),
            (12, -15.38, -2.4, 10),
            (14, -19.6, -2.3, 10), # P1
            (14, -24.23, -2.3, 8), # C1
            (17.69, -28.19, -2.4, 10),
            (20, -32, -2.8, 13),
            (21, -34.70, -2.4, 15),
            (22, -38.13, -2.3, 13),
            (22.5, -41.73, -2.3, 10), # P2
            (21.5, -45.6, -2.2, 8), # C2
            (22, -48.92, -1.8, 10),
            (21, -52.61, -1.7, 13),
            (21, -55, -1.6, 15),
            (20, -58, -1.5, 13),
            (19, -61, -1.4, 13), # P3
            (18, -66, -1, 13), # C3
            (16, -68.5, 0, 13),
            (16, -70, 1, 13),
            (15, -74, 0, 13), #y
            (12, -75, 0, 10),
            (11, -80, 0, 10), # P4
            (10, -85.5, 0, 8), # C4
            (9, -87.5, 0, 10),
            (7, -90, -1, 10),
            (2, -92, -1, 15),
            (-2, -93, -1, 13),
            (-7, -95, -0.5, 10), # P5
            (-13, -93, 0, 8), # C5
            (-16, -94, 0, 8)
        )

        self.circle_yaw_rotateToYawAsync = (
            -90,
            -90,
            -100,
            -120,
            -170,
            -170,
            -180
        )

        self.land_setpoint_moveToPositionAsync = (-62.9, -102.3, -4, 3)
    def get_point(self, id):
        return self.point_moveToPositionAsync[id]

    def get_circle_setpoint(self, id_from_one):
        return self.circle_setpoint_moveToPositionAsync[id_from_one - 1]
    
    def get_circle_targetpoint(self, id_from_one):
        return self.circle_targetpoint_moveToPositionAsync[id_from_one - 1]

    def get_circle_yaw(self, id_from_one):
        return self.circle_yaw_rotateToYawAsync[id_from_one - 1]

    def get_land_setpoint(self):
        return self.land_setpoint_moveToPositionAsync

# ===========================

class airsim_client:
    def __init__(self, ip_addr='127.0.0.1') -> None:
        print("Try to connect {}...".format(ip_addr))
        self.client = airsim.MultirotorClient(ip_addr)
        self.client.confirmConnection()
        self.client.enableApiControl(True)

        # self.dhc = dh.drone_func_class()
        self.circle_finder = circle_finder.circle_finder(self.client)

        self.setpoints = uav_setpoints()

    def task_takeoff(self):
        self.client.armDisarm(True)
        self.client.takeoffAsync(1).join()

    def task_cross_circle(self, circle_id_from_one):
        # self.client.moveToPositionAsync(*self.setpoints.get_circle_setpoint(circle_id_from_one)).join()
        # self.client.hoverAsync().join() # 悬停函数
        # time.sleep(5) # 这个停顿貌似是必要的，如果没有的话会出事
        # airsim.wait_key('Press any key to rotate')
        # self.client.rotateToYawAsync(self.setpoints.get_circle_yaw(circle_id_from_one)).join() # 旋转yaw角，正对障碍物识别
        # time.sleep(3) #
        # circle_xyz = self.circle_finder.get_circle_position_in_wc()
        circle_xyz = self.setpoints.get_circle_targetpoint(circle_id_from_one)
        self.client.moveToPositionAsync(*circle_xyz, 1).join()
        self.client.hoverAsync().join()
        # self.dhc.cross_circle(self.client)

        # self.client.rotateToYawAsync(0).join()
        # time.sleep(3)
    
    def task_land(self):
        self.client.moveToPositionAsync(*self.setpoints.get_land_setpoint()).join()
        self.client.hoverAsync().join() # 悬停函数
        time.sleep(3) #
        self.client.landAsync().join()
        self.client.armDisarm(False)

    def begin_task(self):
        # airsim.wait_key('Press any key to takeoff.')
        print("=========================")
        print("Taking off...")

        self.task_takeoff()
        for point_id in range(len(self.setpoints.point_moveToPositionAsync)):
            self.client.moveToPositionAsync(*self.setpoints.get_point(point_id)).join()
        self.client.hoverAsync().join() # 悬停函数
        '''
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
        '''
