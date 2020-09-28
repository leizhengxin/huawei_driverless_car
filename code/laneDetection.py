#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import cv2
import os
import sys
import glob
import numpy as np
import math

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge


#距离映射
x_cmPerPixel = 90/665.00
y_cmPerPixel = 81/680.00
roadWidth = 665

y_offset = 50.0 #cm

#轴间距
I = 58.0
#摄像头坐标系与车中心间距
D = 18.0
#计算cmdSteer的系数
k = -32

class camera:
    def __init__(self):

        self.camMat = []
        self.camDistortion = []

        self.cap = cv2.VideoCapture('/dev/video10')
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        # 定义参数形式
        self.imagePub = rospy.Publisher('images', Image, queue_size=1)
        self.cmdPub = rospy.Publisher('lane_vel', Twist, queue_size=1)
        self.cam_cmd = Twist()
        self.cvb = CvBridge()
        
        # 透视变换参数
        # src_points = np.array([[3,570], [387,460], [906,452], [1041,485]], dtype="float32")     #变换点前
        # dst_points = np.array([[266., 686.], [266., 19.], [931., 20.], [931., 701.]], dtype="float32")      #变换点后
        # src_points = np.array([[158,507], [446,422], [841,423], [995,467]], dtype="float32")     #变换点前
        # dst_points = np.array([[266., 686.], [266., 19.], [931., 20.], [931., 701.]], dtype="float32")      #变换点后
        src_points = np.array([[446,507], [446,422], [995,423], [995,507]], dtype="float32")     #变换点前
        dst_points = np.array([[266., 686.], [266., 19.], [931., 20.], [931., 701.]], dtype="float32")      #变换点后
        self.M = cv2.getPerspectiveTransform(src_points, dst_points)     #透视变换函数，M为透视变换矩阵

        self.aP = [0.0, 0.0]
        self.lastP = [0.0, 0.0]
        self.Timer = 0

        # self.angleCoef = 16
        # self.angleMinLeft = 0
        # self.distMinLeft = 100
        # self.angleMinRight = 0
        # self.distMinRight = 100
        self.isblack = 0
        # self.distMinLA=100

        # rospy.Subscriber("/scan", LaserScan, self.lidarcallback)
    
    def __del__(self):
        self.cap.release()


    # def lidarcallback(self,data):
    #     minIndexLeft = np.argmin(data.ranges[320:720])
    #     minIndexRight = np.argmin(data.ranges[720:1120])
    #     self.distMinLA = data.ranges[480:960][np.argmin(data.ranges[480:960])]
    #     self.distMinLeft = data.ranges[320:720][minIndexLeft]
    #     self.distMinRight = data.ranges[720:1120][minIndexRight]
    #     self.angleMinLeft = (minIndexLeft-400)*data.angle_increment
    #     self.angleMinRight = (minIndexRight-400)*data.angle_increment

    """
    雷达循迹
    """

    # def lidarSpan(self):
    #     if (self.distMinLeft >= self.distMinRight):
    #         self.cam_cmd.angular.z = -(self.angleCoef*self.distMinLeft/self.distMinRight-self.angleCoef)
    #         self.cam_cmd.linear.x = 0
    #     else:
    #         self.cam_cmd.angular.z = (self.angleCoef*self.distMinRight/self.distMinLeft-self.angleCoef)
    #         self.cam_cmd.linear.x = 0
    #     # if (self.distMinLeft < 0.4 and self.distMinRight < 0.4 and abs(self.angleMinLeft) < 0.7 and abs(self.angleMinRight < 0.7)):
    #     #     self.cam_cmd.angular.z*=50
    #     #     self.cam_cmd.linear.x = -100
    #     self.cam_cmd.angular.z = max(min(self.cam_cmd.angular.z,6),-6)
    #     if self.distMinLA <= 0.5:
    #         self.cam_cmd.linear.y = -100
    #     else:
    #         self.cam_cmd.linear.y = 0
    #     print('distMinLeft:',self.distMinLeft,'distMinRight:',self.distMinRight,'distMinLA:',self.distMinLA)

    def spin(self):
        ret, img = self.cap.read()
        print("ret = ",ret)
        if ret == True:
            gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            kernel = np.ones((3,3), np.uint8)
            gray_img = cv2.erode(gray_img, kernel, iterations=1)        #腐蚀
            origin_thr = np.zeros_like(gray_img)
            # origin_thr[(gray_img >= 125)] = 255
            origin_thr[(gray_img >= 160)] = 255
            
            # 基于直方图的车道线检测
            # src_points = np.array([[3,570], [387,460], [906,452], [1041,485]], dtype="float32")     #变换点前
            # dst_points = np.array([[266., 686.], [266., 19.], [931., 20.], [931., 701.]], dtype="float32")      #变换点后

            binary_warped = cv2.warpPerspective(origin_thr, self.M, (1280, 720), cv2.INTER_LINEAR)
            histogram_x = np.sum(binary_warped[int(binary_warped.shape[0] /2):, :], axis=0)
            
        
            lane_base = np.argmax(histogram_x)              #道路中心
            midpoint_x = int(histogram_x.shape[0]/2)        #视线中心


            histogram_y = np.sum(binary_warped[0:binary_warped.shape[0], :], axis=1)
            midpoint_y = 320 #int(histogram.shape[0]/2)
            upper_half_histSum = np.sum(histogram_y[0:midpoint_y])
            lower_half_histSum = np.sum(histogram_y[midpoint_y: ])
            try:
                hist_sum_y_ratio = (upper_half_histSum)/(lower_half_histSum)
            except:
                hist_sum_y_ratio = 1
            print(hist_sum_y_ratio)


            # 车道线识别
            nwindows = 10
            window_height = int(binary_warped.shape[0] / nwindows)#保证窗口正好覆盖视角
            nonzero = binary_warped.nonzero()#寻找值不为0的点，即寻找图像中值为1的部分
            nonzeroy = np.array(nonzero[0])#行数
            nonzerox = np.array(nonzero[1])#列数
            lane_current = lane_base#边线位置
            margin = 100
            minpix = 25

            lane_inds = []
            ####################取滑动窗口#######################
            for window in range(nwindows):
                win_y_low = binary_warped.shape[0] - (window + 1) * window_height
                win_y_high = binary_warped.shape[0] - window * window_height
                win_x_low = lane_current - margin
                win_x_high = lane_current + margin
                good_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_x_low) & (
                            nonzerox < win_x_high)).nonzero()[0]        #找到滑动窗口内点作为拟合参数备选点

                lane_inds.append(good_inds)
                if len(good_inds) > minpix:
                    lane_current = int(np.mean(nonzerox[good_inds]))
                elif window>=3:#指到了3个窗口仍然没有出现以上的判断
                    break

            lane_inds = np.concatenate(lane_inds)       #拼接

            pixelX = nonzerox[lane_inds]
            pixelY = nonzeroy[lane_inds]                #收集所有滑动窗口范围内点的横纵坐标
            if len(nonzerox) <= 4000:
                # self.lidarSpan()
                # steerAngle = self.cam_cmd.angular.z / k
                self.isblack = 1
                self.cam_cmd.linear.y = -100 # 将开始使用雷达避障
            else:
            ##################曲线拟合####################
                self.isblack = 0
                self.cam_cmd.linear.y = 0
                if (pixelX.size == 0):
                    return
                a2, a1, a0 = np.polyfit(pixelY,pixelX, 2)
                
                frontDistance = np.argsort(pixelY)[int(len(pixelY) / 8)]        #有待商榷
                aimLaneP = [pixelX[frontDistance], pixelY[frontDistance]]       #白线的目标点
                    

                # 计算aimLaneP处斜率，从而得到目标点的像素坐标
                lanePk = 2 * a2 * aimLaneP[1] + a1
                if(abs(lanePk)<0.1):
                    if lane_base >= midpoint_x:
                        if hist_sum_y_ratio < 0.1:
                            LorR = -1.5                         #有待商榷，可能0更好
                            # LorR = 0
                        else:
                            LorR = -1.5
                    else:
                        if hist_sum_y_ratio < 0.1:
                            #LorR = -1.25                         #有待商榷，可能0更好
                            LorR = 0
                        else:
                            LorR = 1.5
                                    
                    self.aP[0] = aimLaneP[0] +LorR * roadWidth / 2
                    self.aP[1] = aimLaneP[1]
                else:
                    y_intercept = a2 * 720 * 720 + a1 * 720 + a0
                    if (y_intercept > 640): #599
                        LorR = -1.8# RightLane
                    else:
                        LorR = 1.8 # LeftLane
                    self.aP[0] = aimLaneP[0] +LorR * roadWidth / 2
                    self.aP[1] = aimLaneP[1]

                self.aP[0] = (self.aP[0] - 640) * x_cmPerPixel #599
                self.aP[1] = (720 - self.aP[1]) * y_cmPerPixel + y_offset #680

                # 计算目标点的真实坐标
                if (self.lastP[0] > 0.001 and self.lastP[1] > 0.001):
                    if (((self.aP[0] - self.lastP[0]) ** 2 + (
                            self.aP[1] - self.lastP[1]) ** 2 > 2500) and self.Timer < 2):  # To avoid the mislead by walkers
                        self.aP = self.lastP[:]
                        self.Timer += 1
                    else:
                        self.Timer = 0

                self.lastP = self.aP[:]

                steerAngle = math.atan(2 * I * self.aP[0] / (self.aP[0] * self.aP[0] + (self.aP[1] + D) * (self.aP[1] + D)))
                self.cam_cmd.angular.z = k * steerAngle

            print('isblack:',self.isblack)
            # self.cam_cmd.angular.z = k * steerAngle             #k值正负可能改变
            self.cmdPub.publish(self.cam_cmd)
            self.imagePub.publish(self.cvb.cv2_to_imgmsg(binary_warped))  # binary_warped
            # cv2.circle(binary_warped,(int(self.aP[0]),int(self.aP[1])),20,(255,255,255),-1)
            print('ap0,ap1',self.aP[0],self.aP[1])
            cv2.imshow('binary_warped', binary_warped)
            cv2.waitKey(1)



if __name__ == '__main__':
    rospy.init_node('lane_vel', anonymous=True)
    rate = rospy.Rate(10)
    
    try:
        cam = camera()
        print(rospy.is_shutdown())  # FALSE
        while not rospy.is_shutdown():
            cam.spin()
            print('betweeen == cam.spin ==')
            rate.sleep()
    except rospy.ROSInterruptException:
        print(rospy.ROSInterruptException)
        pass


