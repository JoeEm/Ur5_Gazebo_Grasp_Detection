#!/usr/bin/env  python
#coding:utf-8
import rospy
import sys
import roslib
from srv/pose_result.srv import *

def client_srv():
    rospy.init_node('pose_result_client')
    #   等待有可用的服务"pose_result_client"
    rospy.wait_for_service("image_pose_result")
    try:
 # 定义service客户端，service 名称为 “pose_result_client”，service 类型为 pos_result.srv
        greetings_client = rospy.ServiceProxy("image_pose_result",pose_result)
 # 向server端发送请求,发送的request内容为 请求确定位（bool值）
 # 此处发送的 request 内容与 srv 文件中定义的 request 部分的属性是一致的
        #resp = greetings_client("HAN",20)
        resp = greetings_client.call(True) #resp is the data sent back from server
        rospy.loginfo("the pose x_1:%f, x_2:%f, %fx_3", %(resp.pox_x, resp.pox_y, resp.pox_z))
        rospy.loginfo("the angle is %f, %f, %f", %(resp.angle_1, resp.angle_2, resp.angle_3))
    except rospy.ServiceException, e:
        rospy.logwarn("Service call failed: %s"%e)

if __name__=="__main__": 
     client_srv()

