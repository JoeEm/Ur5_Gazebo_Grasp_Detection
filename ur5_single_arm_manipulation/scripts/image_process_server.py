#!/usr/bin/env  python
# coding=utf-8
import rospy
import sys
import roslib
from pose_result.srv import *

box1_x = 0.85
box1_y = 0.0
# size of the box in its urdf is 0.045
box1_z = 0.745+0.03
angle_1 = 0
angle_2 = 0
angle_3 = 0


def server_srv():
    # 初始化节点,命名为"image_pose_result"
    rospy.init_node("image_pose_result")
    # 定义service的server端,service名称为"greetings",service类型为pose_result.srv
    # 收到的request请求信息将作为参数传递给handle_function进行处理
    s = rospy.Service("image_pose_result", pose_result1, handle_function)

    rospy.spin()


def handle_function(req):
    if req.request_for_result == True:
        rospy.loginfo("Ready to handle the request:")
        # this funciton_name is created by the sys automatically
        return image_pose_result1Response(box1_x, box1_y, box1_z, angle_2, angle_3)
    else: pass


if __name__ == "__main__":
    server_srv()
