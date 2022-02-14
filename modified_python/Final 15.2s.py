# 全部成功
from airsim.types import ImageRequest, Quaternionr, Vector3r
import airsim


class uav_setpoints:
    def __init__(self) -> None:
        self.client = None
        self.point_moveToPositionAsync = [


            ##=====================================
            ##the position of circles and destion
            ##( 15.8 , - 22.6 , -4.8 ) circle1
            ##( 22.8 , - 45.2 , -4.5 ) circle2
            ##( 19.8 , - 65.4 , -3.7 ) circle3
            ##(  8.0 , - 82.2 , -2.5 ) circle4
            ##(-11.3 , - 93.0 , -2.9 ) circle5
            ##(-29.9 , - 98.6 , -4.7 ) circle6
            ##(-52.1 , -103.0 , -5.7 ) circle7
            ##(-62.9 , -102.3 , -3.0 ) destination
            ##=====================================

            (14.2 , - 22.6, -10.5, 30 ),  ## pass circle1
            (20.7 , - 46.4, -0.2 , 30 ),  ## pass circle2

            ## height is low ; need improvement

            (20.3 , - 60.4, -8   , 20 ),  ## pass circle3
            (9.5  , - 77.9, -0.2 , 20 ),  ## pass circle4
            (-11.3, - 93.0, -2.9 , 20 ),  ## pass circle5
            (-31.9, - 95.9, -1.5 , 20 ),  ## pass circle6
            #keep the velocity following a straight lane , the machine will get accelerated velocity
            (-52.1, -103.0, -3.7 , 10 ),  ## pass circle7
            (-61.0, -96.5 ,  3.0 , 10 ),  ## reach destination


        ]

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
