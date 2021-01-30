#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
from math import *
from geometry_msgs.msg import Pose


class Image_converter:
    def __init__(self):
        self.image_pub = rospy.Publisher(
            'table_detect_test', Image, queue_size=10)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            '/camera_kinect_head_mount/color/image_raw', Image, self.callback)

        ## Set the 
        self.

    def callback(self, data):
        try:
                cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
                #print data
        except CvBridgeError, e:
            print e
        size = self.detect_table(cv_image)
        detect_image = self.detect_box(
            size[0], size[1], size[2], size[3], size[4])
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(detect_image, 'bgr8'))
        except CvBridgeError, e:
            print e

    def detect_table(self, image):
        b, g, r = cv2.split(image) #分离出b,g,r三个通道的信息，opencv中 rgb顺序是反过来的
        binary_image = cv2.medianBlur(r, 3)  #medianblur 中值滤波操作
        for i in range(binary_image.shape[0]): # 行
            for j in range(binary_image.shape[1]): #列
                editValue = binary_image[i, j]
                editValue2 = g[i, j]
                if editValue >= 0 and editValue < 255 and editValue2 >= 0 and editValue2 < 255:#进一步过滤筛选。
                    binary_image[i, j] = 255
                else:
                    binary_image[i, j] = 0

        img, cnts, hierarchy = cv2.findContours(  #找到图的（多个可能轮廓）
            binary_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        x, y, w, h = cv2.boundingRect(binary_image) #返回一个矩形框，左上角为原点，
        cv2.rectangle(image, (x, y), (x+w, y+h), (0, 0, 255), 4) #按照bgr的配置去画出框框， 最后一个为框宽度
        # loop over the contours
        for c in cnts: #可能不止于一个物体，所以是个轮廓链表
            # compute the center of the contour
            M = cv2.moments(c) 

            if int(M["m00"]) not in range(20000, 250000):
                continue

            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            #这里将重心坐标cX,cY打印出来
            #print()
            #rospy.loginfo("桌子的重心坐标为：(%f,%f)" %(cX, cY))
            cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)
        return image, x, y, w, h

    def detect_box(self, image, x, y, w, h):
        b, g, r = cv2.split(image)
        binary_image = cv2.medianBlur(g, 3)
        for i in range(binary_image.shape[0]):
            for j in range(binary_image.shape[1]):
                editValue = binary_image[i, j]
                if i < y or i > y+h or j < x or j > x+w:
                    binary_image[i, j] = 0
                else:
                    if editValue > 0 and editValue <= 30:
                    #if editValue > 120 and editValue <= 255:
                        binary_image[i, j] = 255
                    else:
                        binary_image[i, j] = 0
        img, cnts, hierarchy = cv2.findContours(
            binary_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(image, cnts, -1, (255, 0, 0), 2) #画出物体（box）的轮廓
        rospy.loginfo("物体的重心坐标为:")
        for c in cnts:
            # compute the center of the contour
            M = cv2.moments(c)

            if int(M["m00"]) not in range(1, 250000):
                continue

            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            #打印box的重心cX,CY
            rospy.loginfo("物体的重心坐标为：(%f,%f)" %(cX, cY))
            #print(cX)
            #print(cY)
            cv2.circle(image, (cX, cY), 5, (255, 0, 0), -1)
        return image


if __name__ == "__main__":
    rospy.init_node("vision_manager")
    rospy.loginfo("start")
    Image_converter()
    rospy.spin()
