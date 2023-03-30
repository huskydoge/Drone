

import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R 
import math

class circle_finder:
    def __init__(self, airsim_client) -> None:
        self.client = airsim_client
        self.cx = 320
        self.cy = 240
        self.fx = 268.5
        self.fy = 268.5

    def get_circle_x_y_z(self, rgb, depthperspective, depth_lower_bound, depth_higher_bound, circle_number = 1):
        hue_image = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
        shape = hue_image.shape
        low_range = np.array([150, 20, 10])
        high_range = np.array([180, 255, 255])
        light_range= np.array([6,8])
        store=[] #用于储存像素坐标
        gray = np.array([[0]*shape[1] for i in range(shape[0])]).astype(np.uint8)
        if circle_number == 4:
            light_range= np.array([6,8])
            for i in range(shape[0]):
                for j in range(shape[1]):
                    if ((((hue_image[i][j][0] > low_range[0]) and  (hue_image[i][j][0] < high_range[0])) | ((hue_image[i][j][0] > light_range[0]) and  (hue_image[i][j][0] < light_range[1]) ))
                        and (hue_image[i][j][1] > low_range[1]) and  (hue_image[i][j][1] < high_range[1])
                        and (hue_image[i][j][2] > low_range[1]) and  (hue_image[i][j][2] < high_range[1])
                        and (depthperspective[i][j] >= depth_lower_bound) 
                        and (depthperspective[i][j] <= depth_higher_bound)):
                        gray[i][j] = 255
                        store.append([i,j])
                    else:
                        gray[i][j] = 0
        else:                
            for i in range(shape[0]):
                for j in range(shape[1]):
                    if (((hue_image[i][j][0] > low_range[0]) and  (hue_image[i][j][0] < high_range[0]))
                        and (hue_image[i][j][1] > low_range[1]) and  (hue_image[i][j][1] < high_range[1])
                        and (hue_image[i][j][2] > low_range[1]) and  (hue_image[i][j][2] < high_range[1])
                        and (depthperspective[i][j] >= depth_lower_bound) 
                        and (depthperspective[i][j] <= depth_higher_bound)):
                        gray[i][j] = 255
                        store.append([i,j])
                    else:
                        gray[i][j] = 0
        # cv2.imshow("gray", gray)
        # cv2.waitKey(0)



        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 50, param1=1, param2=1, minRadius=0, maxRadius=60)
        
        circles = list(circles) #
        circles.sort(key = lambda x:x[2], reverse = True) #找出一次识别中半径最大的那个圆，也就是我们的目标圆
        circle = circles[0][0]#加入这个圆的信息[x, y, r]
        x = int(circle[0])
        y = int(circle[1])
        r = int(circle[2])

        # # 4 将检测结果绘制在图像上
        # cv2.circle(rgb, (x, y), r, (0, 255, 255), 2)
        # cv2.imshow("result", rgb)
        # cv2.waitKey(0)


        tmp = []
        for i,j in store:
            k1 = (j - self.cx) / self.fx
            k2 = (i - self.cy) / self.fy
            z = depthperspective[i][j] / math.sqrt(k1**2 + k2**2 + 1) 
            tmp.append(z)

        circle_center_z = sum(tmp) / len(tmp)
        return circle[0], circle[1], circle_center_z # 输出检测到的圆的x, y, z坐标


    def circle_cc_to_wc(self, pixel_x, pixel_y, z, t, R):

        camera_inner_matrix = [[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1]]
        camera_inner_matrix = np.linalg.pinv(np.array(camera_inner_matrix))

        point2D_h = [pixel_x, pixel_y, 1]
        point = (np.array(point2D_h) * z).T

        tmp = np.dot(camera_inner_matrix, point)
        tmp[2] += 0.6

        self.R_b_c = [[0, 0 ,1], [1, 0, 0], [0, 1, 0]]
        tmp = np.dot(self.R_b_c, tmp)
        print("circle in body coord", tmp)
        result = np.dot(R, tmp) + np.array(t).T
        result = list(result)
        return result

    def get_circle_position_in_wc(self,pics_rgb, depthperspective,position_list, rotation_matrix, depth_lower_bound, depth_highter_bound, circle_number = 1):
        if circle_number == 4:
            circle_xyz = self.get_circle_x_y_z(pics_rgb, depthperspective, depth_lower_bound, depth_highter_bound,4 )
            result = self.circle_cc_to_wc(circle_xyz[0], circle_xyz[1], circle_xyz[2], position_list, rotation_matrix)
            return result
        else:
            circle_xyz = self.get_circle_x_y_z(pics_rgb, depthperspective, depth_lower_bound, depth_highter_bound)
            result = self.circle_cc_to_wc(circle_xyz[0], circle_xyz[1], circle_xyz[2], position_list, rotation_matrix)
            return result



