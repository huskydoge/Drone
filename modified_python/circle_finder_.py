
import airsim
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R 
import math

#wc----世界坐标
class circle_finder:
    def __init__(self, airsim_client) -> None:
        self.client = airsim_client #连接无人机

        self.cx = 320  #像素平面比之成像平面在x轴方向平移的量
        self.cy = 240   #像素平面比之成像平面在y轴方向平移的量
        self.fx = 268.5 # fx = 像素平面和成像平面在x轴方向伸缩 a 乘上焦距
        self.fy = 268.5 # fy = 像素平面和成像平面在y轴方向伸缩 b 乘上焦距

    def get_uav_position_rotation_in_wc(self):
        state = self.client.getMultirotorState() #得到无人机位姿和状态
        quaternionr = state.kinematics_estimated.orientation #获取姿态角
        w = quaternionr.w_val 
        x = quaternionr.x_val
        y = quaternionr.y_val
        z = quaternionr.z_val
        tmp = [x, y, z, w]
        r = R.from_quat(tmp) #从四元数初始化 
        rotation_matrix = r.as_matrix() #转换为旋转矩阵
        position = state.kinematics_estimated.position #世界坐标
        position_list = []
        position_list.append(position.x_val)
        position_list.append(position.y_val)
        position_list.append(position.z_val)
        
        return position_list, rotation_matrix

    def get_rgb_depthperspective_image(self):
        depthperspective = self.client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.DepthPerspective, pixels_as_float =True , compress = True)])[0]
        #第一个Type为scene的是rgb三色图，三维
        # 第二个是深度图，二维 
        # compress = False 无压缩
        # pics_rgb = png_image[0]
        #Pfm（或 Portable FloatMap）图像格式将图像存储为浮点像素，因此不限于通常的 0-255 像素值范围。这对于 HDR 图像或描述颜色以外的内容（如深度）的图像很有用。
        depthperspective = airsim.get_pfm_array(depthperspective)
        return  depthperspective

    def get_circle_x_y(self, depthperspective):
        depthperspective[depthperspective > 8] = 0 #depthperspective中大于8的值全变为0，numpy语法 
        depthperspective = depthperspective.astype(np.uint8) #8位无符号整型，0～255
        depthperspective = cv2.equalizeHist(depthperspective) #对灰度图进行非线形拉伸，提高图像对比度，提高骨架结构的显示效果
        # print(depthperspective)
        # cv2.imshow('depth', depthperspective)
        # cv2.waitKey(1000)
        # count = 0
        # circle = [0, 0, 0]
        # 霍夫变换圆检测 得到相机坐标系下的圆心坐标
        circles = None
        while circles is None:
            circles = cv2.HoughCircles(depthperspective, cv2.HOUGH_GRADIENT, 1, 
                                        30, param1=None, param2=30, minRadius=30, maxRadius=300) # 注意图片分辨率大小与圆半径检测

        #由于霍夫圆检测对噪声比较敏感，所以可以考虑对图像进行中值滤波
        '''
        circles = cv.HoughCircles(image, method, dp, minDist, param1=100, param2=100, minRadius=0,maxRadius=0 )
        image:输入图像, 应输入灰度图像

        method: 使用霍夫变换圆检测的算法, 它的参数是CV_HOUGH_GRADIENT

        dp:霍夫空间的分辨率, dp=1时表示霍夫空间与输入图像空间的大小一致, dp=2时霍夫空间是输入图像空间的一半, 以此类推

        minDist为圆心之间的最小距离, 如果检测到的两个圆心之间距离小于该值, 则认为它们是同一个圆心

        param1: 边缘检测时使用Canny算子的高阈值, 低阈值是高阈值的一半。

        param2: 检测圆心和确定半径时所共用的阈值

        minRadius和maxRadius为所检测到的圆半径的最小值和最大值, 如果超出的话这个圆会被舍弃,但是。。。。

        返回: 
        circles:输出圆向量, 包括三个浮点型的元素——圆心横坐标, 圆心纵坐标和圆半径, 是一个三维数组, 比如:
        array([[[494.5, 820.5,  71.7],
                [494.5, 596.5,  71.3],
                [221.5, 370.5,  72. ],
                [774.5, 370.5,  72. ],
                [221.5, 596.5,  71.2],
                [773.5, 820.5,  71.6],
                [493.5, 369.5,  70.9],
                [220.5, 820.5,  71.5],
                [774.5, 594.5,  70.6],
                [362.5, 145.5,  64.5],
                [632.5, 138.5,  63.3]]], dtype=float32)
        
        '''

        #从numpy转化为list才能sort
        circles = list(circles) #
        circles.sort(key = lambda x:x[2], reverse = True) #找出一次识别中半径最大的那个圆，也就是我们的目标圆

        # circle = circles[0][0]#加入这个圆的信息[x, y, r]
        # count += 1
        # if(count >= 10):
        #     circle = circle / count 
        # for circle in circles[0]:
        # x = int(circle[0])
        # y = int(circle[1])
        # r = int(circle[2])
        # img1d = np.frombuffer(rgb.image_data_uint8, dtype=np.uint8) 
        # img_rgb = img1d.reshape(rgb.height, rgb.width, 3)
        # rgb_pic = cv2.circle(img_rgb, (x, y), r, (0, 0, 255), 3) # 显示圆
        # rgb_pic = cv2.circle(rgb_pic, (x, y), 2, (255, 255, 0), -1) # 显示圆心
        # cv2.imshow('new', rgb_pic)
        # cv2.waitKey(1000)
        # cv2.destroyAllWindows()
        return circles[0][0][0], circles[0][0][1] # 输出检测到的圆在像素平面坐标系下的的x, y坐标


    def get_circle_center_z(self, depthperspective):
        # mask = cv2.inRange(depthperspective, 1, 8)
        shape=depthperspective.shape
        # index = np.nonzero() #inRange 让1 ～ 8 的变为 255， 其他变为0
        # starty = (index[0][0]); endy = (index[0][-1])
        # startx = index[1][0] ; endx = index[1][-1]
        # shape = depthperspective.shape
        tmp = []
        for i in range(shape[0]):
            for j in range(shape[1]):
                if depthperspective[i][j] >= 1 and depthperspective[i][j] <= 8:
                    k1 = (j - self.cx) / self.fx
                    k2 = (i - self.cy) / self.fy
                    z = depthperspective[i][j] / math.sqrt(k1**2 + k2**2 + 1) 
                    tmp.append(z)
        circle_center_z = sum(tmp) / len(tmp)
        return circle_center_z

    def circle_cc_to_wc(self, pixel_x, pixel_y, z, t, R): 
        camera_inner_matrix = [[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1]]
        camera_inner_matrix = np.linalg.pinv(np.array(camera_inner_matrix))
        # print('pixels', pixel_x, pixel_y)
        point2D_h = [pixel_x, pixel_y, 1]
        point = (np.array(point2D_h) * z).T
        # print("point2D:", point)
        tmp = np.dot(camera_inner_matrix, point)
        tmp[2] += 0.6
        # print("circle in camera coord", tmp)
        self.R_b_c = [[0, 0 ,1], [1, 0, 0], [0, 1, 0]]
        tmp = np.dot(self.R_b_c, tmp)
        # print("circle in body coord", tmp)
        result = np.dot(R, tmp) + np.array(t).T
        # print("circle in world frame", list(result))
        return list(result)

    def get_circle_position_in_wc(self):
        position_list, rotation_matrix = self.get_uav_position_rotation_in_wc()
        depthperspective = self.get_rgb_depthperspective_image()
        circle_xy = self.get_circle_x_y(depthperspective)
        # print("circle_xy", circle_xy)
        circle_z = self.get_circle_center_z(depthperspective)
        result = self.circle_cc_to_wc(circle_xy[0], circle_xy[1], circle_z, position_list, rotation_matrix)
        return result


