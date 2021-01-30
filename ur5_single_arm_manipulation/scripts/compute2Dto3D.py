#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import copy
import numpy as np
import os

############pos of ee_link when the robot at default pos#########
#position: 
#position: 
#  x: 0.834176942748
#  y: 0.109119718195
#  z: 1.15178749326
#orientation: 
#  x: 2.68088220677e-05
#  y: 0.7071929293
#  z: 5.30219794366e-05
#  w: 0.70702062008

#1/dx
#1/dy 
offset = 0.08
Kinect_x = 0.834176942748 + offset
Kinect_y = 0.109119718195
Kinect_z = 1.15178749326



#cX = 406
#cY = 281

#cX = 556
#cY = 281
###### (when kinect's pos is at{2, 0.0 , 1})
cX = 556.00
cY = 280.00  
#cY=425



table_z = 0.40
#Zc = 0.865


f = 554.254691191187

Zc = (Kinect_z-0.40)*f





#转换从 像素坐标系-> 相机坐标系
pixelVector = [[cX], [cY], [1]]
InnerParam_Of_camera =  [ [554.254691191187, 0.0,              320.5 ],  
                         [0.0,              554.254691191187,  240.5 ], 
                         [0.0,              0.0,               1.0]  ]


#R1 =                      [ [1.0,              0.0,             0.0  ],    
#                          [0.0,                 -1.0,            0.0 ], 
#                          [0.0,              0.0,              -1.0] ]

#R2 =                      [ [0.7071,            0.0,            0.7071  ],     #-PI/4
#                          [0.0,                 1.0,            0.0 ], 
#                          [-0.7071,              0.0,            0.7071] ]

R = 2

Rotation_3 =  [ [0.7071,            0.0,            -0.7071  ],      # rotate -45 degree by Y
              [0.0,                 1.0,            0.0 ], 
              [0.7071,              0.0,            0.7071] ]


Rotation_2 =  [ [1.0,              0.0,               0.0 ],        # rotate -180 degree by X
              [0.0,              -1.0,                0.0 ],
              [0.0,              0.0,                 -1.0] ]

Rotation_1 =  [ [0.0,              -1.0,             0.0 ],         # rotate -90 degree by Z
              [  1.0,              0.0,              0.0 ], 
              [  0.0,              0.0,              1.0] ]

if (R == 3):
    Rotation  = np.dot(np.dot(Rotation_3, Rotation_2), Rotation_1)
else:
    Rotation  = np.dot(Rotation_2, Rotation_1)

World_x = 2.5
World_y = 0.2
World_z = 0

InnerParam_Of_camera_Augmented =  [ [554.254691191187, 0.0,              320.5 , 0.0],  
                                  [0.0,              554.254691191187,  240.5 ,  0.0], 
                                  [0.0,              0.0,               1.0,     0.0] ]

Outer_Param_Matrix =  [            [1,            0.0,               0.0,        Kinect_x],  
                                   [0.0,          -1.0,              0.0,        Kinect_y], 
                                   [0.0,          0.0,               -1.0,       Kinect_z], 
                                   [0.0,          0.0,               0.0,         1.0] ]

WordVector = [ [World_x], [World_y], [World_z], [1] ]

Answer_Right_Side = np.dot(np.dot(InnerParam_Of_camera_Augmented, Outer_Param_Matrix), WordVector)
print("Answer right side:")
print(Answer_Right_Side)
print(Answer_Right_Side[0,0])

Zc1 = Answer_Right_Side[0,0]/cX
Zc2 = Answer_Right_Side[1,0]/cY

print("Zc1 = ")
print Zc1
print("Zc2 = ")
print Zc2



#Rotation =   np.dot(R2, R1)




TransitionVector = [ [Kinect_x], 
                     [Kinect_y],
                     [Kinect_z] ]
                          

inv1 = np.linalg.inv(InnerParam_Of_camera)
#图像坐标
ImageVector =  np.dot(inv1, pixelVector)
ImageVector = ImageVector
print('图像坐标系的值:')
print (ImageVector)




New_pixelVector = ImageVector
#New_pixelVector = np.dot(Zc, ImageVector)

print('相机坐标系的值x,y:')
Cam_x = ImageVector[0,0]*(Zc/f)
Cam_y = ImageVector[1,0]*(Zc/f)
print (Cam_x)
print (Cam_y)

CameraVector = [[Cam_x], [Cam_y], [1]]


inv_R = np.linalg.inv(Rotation)

#世界坐标系
WorldVector = np.dot(Rotation, CameraVector) + TransitionVector


print('世界坐标系的值：')
print (WorldVector)