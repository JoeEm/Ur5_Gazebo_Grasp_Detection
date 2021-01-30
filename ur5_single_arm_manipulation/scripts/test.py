#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Quaternion
import std_msgs.msg
import json
import jsonFileTest
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String

import sys
import copy
import numpy
import os

from numpy import *



Grasp_pose_use= Pose()
#Grasp_pose_use.header.frame_id = '/world'
Grasp_pose_use.position.x = 1
Grasp_pose_use.position.y = 2 
Grasp_pose_use.position.z = 3 #- grasp_z_err #- Base_Height# base_link as a reference frame
q_grasp = quaternion_from_euler(1.57, 1.57, 1.57) #Q number


print(  euler_from_quaternion( Grasp_pose_use.orientation) )



place_Pose_use = Pose()
place_Pose_use.position.x = box1_x #+ 0.1
place_Pose_use.position.y = box1_y -0.3 #- 0.4
place_Pose_use.position.z = box1_z + Place_Z_OFFSET  # place the stuff a little higher above the table (world axis as a reference)
q = quaternion_from_euler(1.57, 1.57, 0) #Q number
place_Pose_use.orientation = Quaternion(*q)   



"""
{
    "target_lable":"螺母",
    "target_pose":"(0.86,0,0.545)",
    "target_angle":"(1.57, 1.57, 0)",  object A 
}
{
    "target_lable":"螺栓",
    "target_pose":"(0.86,-0.20,0.645)", object B
    "target_angle":"(1.57, 1.57, 1.57)",
}
"""

json_str = '''
{
    "response_code":200,
    "param_entity":[
        {
            "target_lable":"螺母",
            "target_pose":"(x,y,z)",
            "target_angle":"(a,b,c)"
        },
        {
            "target_lable":"螺栓",
            "target_pose":"(x,y,z)",
            "target_angle":"(a,b,c)"
        }
    ]
}
'''




x1 = 0.86
print(type(x1))
print(str(0.86))
print(type(str(0.86)))


JasonDict  = json.loads(json_str)
print(type(JasonDict["param_entity"][0]["target_pose"]))

print()

a = str(JasonDict["param_entity"][0]["target_pose"])
print(a)
print(type(a))

b = a.replace("x",str(x1))
print(b)

print(len(JasonDict["param_entity"]))


m = 2 

for a in range(0,m):
	print(a)


### Fill the Pose() Type data into 2 arrays separately #pos[0]
pos = [1, 2, 3]
angle = [1.57,1.57,1.57]

print(angle[0])

b = random.rand(2,3)

for row in range(0, b.shape[0]):
    for col in range(0, b.shape[1]):
        if   (row == 0):
          b[row,col]  = pos[col]

        elif (row == 1):
           b[row,col] = angle[col]
###
print("testting the list")
print b

