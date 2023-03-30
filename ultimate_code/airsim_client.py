
import airsim
import time
import numpy as np
from scipy.spatial.transform import Rotation as R 
import circle_finder


class uav_setpoints:
    def __init__(self) -> None:
        self.circle_setpoint_moveToPositionAsync = (
            (15.5, -19.6 , -3, 5),
            (22, -41.2 , -2.5, 5),
            (21, -61.5 , -2, 3),
            (10, -78.2, -2, 3),
            (-9.3, -93, -2.5, 3),
            (-27, -98, -4, 3),
            (-50.1, -103, -5.7, 3)
        )

        self.mid_point = (
            (10.5, -13.5, -3.50, 10),
            (20, -33, -2, 10),
            (20.5, -61.5, -1.6, 3),
            (14.5, -75, -0.8, 10),
            (-6.5, -92, -1.13, 6),
            (-20.5, -97.5, -1.5, 10),
            (-47, -102.5, -5, 6)
        )
#5 
        self.circle_yaw_rotateToYawAsync = (
            -90,
            -90,
            -100,
            -120,
            -170,
            -170,
            -180
        )

        self.land_setpoint_moveToPositionAsync = (-62.9, -102.3, -4, 0)

    def get_circle_setpoint(self, id_from_one):
        return self.circle_setpoint_moveToPositionAsync[id_from_one - 1]
    
    def get_midpoint(self, id_from_one):
        return self.mid_point[id_from_one - 1]

    def get_circle_yaw(self, id_from_one):
        return self.circle_yaw_rotateToYawAsync[id_from_one - 1]

    def get_land_setpoint(self):
        return self.land_setpoint_moveToPositionAsync

# ===========================

class airsim_client:
    def __init__(self, ip_addr='127.0.0.1') -> None:
        circle_xyz = (0,0,0)
        print("Try to connect {}...".format(ip_addr))
        self.client = airsim.MultirotorClient(ip_addr)
        self.client.confirmConnection()
        self.client.enableApiControl(True)

        self.circle_finder = circle_finder.circle_finder(self.client)

        self.setpoints = uav_setpoints()

    def task_takeoff(self):
        self.client.armDisarm(True)
        self.client.takeoffAsync(0.01).join()

    def get_uav_position_rotation_in_wc(self):
        begin = time.time()
        state = self.client.getMultirotorState()
        quaternionr = state.kinematics_estimated.orientation
        w = quaternionr.w_val
        x = quaternionr.x_val
        y = quaternionr.y_val
        z = quaternionr.z_val
        tmp = [x, y, z, w]
        r = R.from_quat(tmp)
        rotation_matrix = r.as_matrix()
        position = state.kinematics_estimated.position
        position_list = []
        position_list.append(position.x_val)
        position_list.append(position.y_val)
        position_list.append(position.z_val)
        end = time.time()
        print("position and rotation get in {}".format(end - begin))
        return position_list, rotation_matrix

    def get_rgb_depthperspective_image(self):
        begin = time.time()
        png_image = self.client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, pixels_as_float = False, compress = False),
                                        airsim.ImageRequest("0", airsim.ImageType.DepthPerspective, pixels_as_float = True, compress = False)])

        pics_rgb = png_image[0]
        img1d = np.frombuffer(pics_rgb.image_data_uint8, dtype=np.uint8) 
        img_rgb = img1d.reshape(pics_rgb.height, pics_rgb.width, 3)
        # cv2.imshow("RGB",img_rgb)
        # cv2.waitKey(100000)
        depthperspective = png_image[1]
        depthperspective = airsim.get_pfm_array(depthperspective)
        end = time.time()
        print("depth and pics get in {}".format(end-begin))
        return img_rgb, depthperspective
    
    def move(self,position):
        self.client.moveToPositionAsync(*position).join()

    def task_cross_circle(self):
        # 1
        self.client.moveToPositionAsync(*self.setpoints.get_midpoint(1)).join()
        # 旋转yaw角，正对障碍物识别   复原yaw角
        self.client.rotateToYawAsync(self.setpoints.get_circle_yaw(1)).join() # 旋转yaw角，正对障碍物识别
        self.client.hoverAsync().join()
        #获取旋转矩阵、位置、两种图片
        pics_rgb, depthperspective = self.get_rgb_depthperspective_image()
        position_list, rotation_matrix = self.get_uav_position_rotation_in_wc() #所需时间很少
        #计算坐标
        circle_1_xyz = circle_finder.circle_finder(airsim.MultirotorClient('127.0.0.1')).get_circle_position_in_wc(pics_rgb, depthperspective,position_list, rotation_matrix, 0, 9)
        circle_2_xyz = circle_finder.circle_finder(airsim.MultirotorClient('127.0.0.1')).get_circle_position_in_wc(pics_rgb, depthperspective,position_list, rotation_matrix, 25, 35)
        self.client.moveToPositionAsync(*circle_1_xyz, 4).join()


        # 2
        circle_2_xyz[2] -= 0.45 # 调整高度
        circle_2_xyz[0] -= 1.8
        self.client.moveToPositionAsync(*circle_2_xyz, 20).join()


        # 3
        self.client.moveToPositionAsync(*self.setpoints.get_midpoint(3)).join()
        self.client.rotateToYawAsync(0).join()
        self.client.hoverAsync().join()
        self.client.rotateToYawAsync(self.setpoints.get_circle_yaw(3)).join()
        pics_rgb, depthperspective = self.get_rgb_depthperspective_image()
        position_list, rotation_matrix = self.get_uav_position_rotation_in_wc()
        circle_3_xyz = circle_finder.circle_finder(airsim.MultirotorClient('127.0.0.1')).get_circle_position_in_wc(pics_rgb, depthperspective,position_list, rotation_matrix, 0, 9)
        circle_4_xyz = circle_finder.circle_finder(airsim.MultirotorClient('127.0.0.1')).get_circle_position_in_wc(pics_rgb, depthperspective,position_list, rotation_matrix, 20, 35, 4)
        self.client.moveToPositionAsync(*circle_3_xyz, 3).join()


        # 4
        circle_4_xyz[2] += 0.5 
        self.client.moveToPositionAsync(*circle_4_xyz, 8).join()

        # 5
        self.client.moveToPositionAsync(*self.setpoints.get_midpoint(5)).join()
        self.client.rotateToYawAsync(0).join()
        self.client.hoverAsync().join()
        self.client.rotateToYawAsync(self.setpoints.get_circle_yaw(5)).join()
        pics_rgb, depthperspective = self.get_rgb_depthperspective_image()
        position_list, rotation_matrix = self.get_uav_position_rotation_in_wc()
        circle_5_xyz = circle_finder.circle_finder(airsim.MultirotorClient('127.0.0.1')).get_circle_position_in_wc(pics_rgb, depthperspective,position_list, rotation_matrix, 0, 9)
        circle_6_xyz = circle_finder.circle_finder(airsim.MultirotorClient('127.0.0.1')).get_circle_position_in_wc(pics_rgb, depthperspective,position_list, rotation_matrix, 20, 25)
        self.client.moveToPositionAsync(*circle_5_xyz, 3).join()

        # 6
        circle_6_xyz[2] -= 0.7 
        circle_6_xyz[0] -= 0.3
        self.client.moveToPositionAsync(*circle_6_xyz, 10).join()

        # 7
        self.client.moveToPositionAsync(*self.setpoints.get_midpoint(7)).join()
        self.client.rotateToYawAsync(0).join()
        self.client.hoverAsync().join()
        self.client.rotateToYawAsync(self.setpoints.get_circle_yaw(7)).join()
        pics_rgb, depthperspective = self.get_rgb_depthperspective_image()
        position_list, rotation_matrix = self.get_uav_position_rotation_in_wc()
        circle_7_xyz = circle_finder.circle_finder(airsim.MultirotorClient('127.0.0.1')).get_circle_position_in_wc(pics_rgb, depthperspective,position_list, rotation_matrix, 0, 9)
        self.client.moveToPositionAsync(*circle_7_xyz, 3).join()

    def begin_task(self):
        print("=========================")
        print("Taking off...")

        self.task_takeoff()
        self.task_cross_circle()
        
        print("=========================")
        print("Landing...")
        self.client.moveToPositionAsync(*(-62.9, -102.3, -2), 15).join()
        print("=========================")
        print("Task is finished.")

