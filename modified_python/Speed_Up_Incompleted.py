# 全部成功
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
            (16, -68.5, -0.5, 13),
            #(15, -69.5, -0.5, 13),
            #(14, -72.5, -0.5, 13), 
            #(12, -74.5, -0.5, 11),
            (10.5, -80, -0.5, 8), # P4  -79.5->-80 ##
            #(6, -8, 0, 8), # C4
            #(5, -86.5, 0, 10),  #y=-87.5,x=9
            #(3, -88, 0, 8),
            #(-1, -90, 0, 8),##y=-91
            #(-4, -91, -0.5, 8),# y=-92
            #(-7, -92, -0.5, 8), # P5
            (-9.5, -92, -1.2, 15), # C5 -9.5, -92, -1.5, 10
            #(-16, -93, -1.5, 8),
            #(-18, -94, -2, 10),
            #(-20, -95, -3, 10),
            (-22, -96, -2.3, 13),#-3
            #(-24, -97, -2.3, 10),#-2.5
            (-27, -98.5, -1.98, 12.8),#-2.5 -97 -100
            (-27, -99, -2.04, 11), # C6 #-2.5 v=10,-98
            (-32, -100.5, -3,11), #v=8
            (-37, -101, -5.5, 11), #v=10 -4.5
            #(-41, -101, -3.5, 10),
            #(-44, -102, -3.5, 8),
            #(-49, -103, -4, 7),
            (-53, -102, -4.62, 9.5),#x=-59,y=-103.5,v=10,z=-5
            (-56.5, -101, 0, 9.6), #x=-51 -103.5
            (-63.5, -92.5, 6, 13),#63.5,99.5
            #(-63.5, -99, 10, 13)#x=-62.5,y=-102.3
        )

        self.land_setpoint_moveToPositionAsync = (-62.9, -102.3, -4, 3)

    def get_point(self, id):
        return self.point_moveToPositionAsync[id]

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
        self.client.takeoffAsync(0.5).join()

    def begin_task(self):
        # airsim.wait_key('Press any key to takeoff.')
        print("=========================")
        print("Taking off...")

        self.task_takeoff()
        for point_id in range(len(self.setpoints.point_moveToPositionAsync)):
            self.client.moveToPositionAsync(*self.setpoints.get_point(point_id)).join()
        
        print("Task is finished.")
