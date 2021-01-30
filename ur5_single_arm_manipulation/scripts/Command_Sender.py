#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from ur5_single_arm_manipulation.msg import CosCommd
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Quaternion
import std_msgs.msg
import json
import jsonFileTest
from tf.transformations import quaternion_from_euler
from std_msgs.msg import String

import sys
import copy
import numpy
import os


########################################################################################
"""
table_x = 0.56 + 0.3
table_z = 0.50
table_y = 0
#table_z = 0.70+0.03  # increase z by 0.03 to make gripper reach block
grasp_z_err = 0.02

gripper_depth = 0.15 #that is taking the gripper length into account

box1_x = table_x
box1_y = table_y
# size of the box in its urdf is 0.045
box1_z = table_z + 0.045
#grasping pose for box1
Base_Height = 0.9
#p1 stands for grasping pose
Grasp_pose_use= Pose()
#Grasp_pose_use.header.frame_id = '/world'
Grasp_pose_use.position.x = box1_x
Grasp_pose_use.position.y = box1_y 
Grasp_pose_use.position.z = box1_z #- grasp_z_err #- Base_Height# base_link as a reference frame
q_grasp = quaternion_from_euler(1.57, 1.57, 1.57) #Q number
Grasp_pose_use.orientation = Quaternion(*q_grasp)   



# setting the placePose after grasping the stuff
Place_Z_OFFSET = 0.1 #if smaller than 0.03
Place_Y_OFFSET = 0.3

place_Pose_use = Pose()
place_Pose_use.position.x = box1_x #+ 0.1
place_Pose_use.position.y = box1_y -0.3 #- 0.4
place_Pose_use.position.z = box1_z + Place_Z_OFFSET  # place the stuff a little higher above the table (world axis as a reference)
q = quaternion_from_euler(1.57, 1.57, 0) #Q number
place_Pose_use.orientation = Quaternion(*q)   
"""


def JsonTypeDataCallback(Data): #Data.data is a jsonType Str

    rospy.loginfo("receiving JsonTypeStr...")

    #create a instance for processing the JosonType str
    String_indicator_1 = jsonFileTest.String_Proccessor(Data.data)
    MaxNum = String_indicator_1._getMaxNum_Of_Sequences()  #calculating the maxNum of the Sequeces
    rospy.loginfo('MaxNum is %d ', MaxNum)


    rospy.loginfo("Juding the DataType : O bject or Tasks Sequences?")
    if (String_indicator_1.Datadict["response_code"] == "200"): #code:200 -> Tasks Sequences

        rospy.loginfo("The Msg is Tasks-Sequences Data.")
        ####Processing the Tasks Sequences
        for num in range(0,MaxNum):#[1,MaxNum+1)
            #while (rospy.get_param('Excute_Lock')==False): #Wait until a sequence is done.
                #rospy.loginfo('waiting to unlock the Movement-mutex ')
            #    pass

            rospy.sleep(2)
            rospy.loginfo("go into loop, num is %d." ,num)
            jsonDataNum = num

            Cmsg = CosCommd()
            rospy.loginfo('jsonDataNum is %d ', jsonDataNum)
            rospy.sleep(5)
            comNum = String_indicator_1.CommandSelection(jsonDataNum) #get the command code 
            Cmsg.ComNum = comNum
        
            Cmsg.GRIPPER_CLOSED = 0.2680
            Cmsg.gripper_depth = 0.148 # 0.145~0.150
            Cmsg.displacementUp = 0.1
            Cmsg.displacementDown = 0.07
            Cmsg.pose = Pose()

            print("num is:")
            print(jsonDataNum)
            if (String_indicator_1._check_Pose_is_needed(jsonDataNum) == True):
                print("command need to calculate the POSE()...")
                Cmsg.pose = String_indicator_1._str2NumForPose(jsonDataNum)
                print("print the pose")
                print(Cmsg.pose)
        

            ComandPublisher.publish(Cmsg)
            rospy.loginfo('msg sent and excute Command-%d',Cmsg.ComNum)
            while (rospy.get_param('Excute_Lock')==False): #Wait until a sequence is done.
                #rospy.loginfo('waiting to unlock the Movement-mutex ')
                pass

if __name__ == '__main__':
    # Set a node As a central Decider. 
    rospy.init_node('Decider')

    # Set a topic that is responsible for sending the Task-Command to the UR_5 Front end. 
    topic_name_Command = '/Decider/CosCmd'
    ComandPublisher = rospy.Publisher(topic_name_Command, CosCommd, queue_size=10)
    rospy.loginfo("...1")
    
    # Set a topic that is responsible for receiving the JsonTypeStr and
    # excute certain operation accordingly.
    topic_name_Json = '/Decider/JsonTypeData/Sequences'
    JsonSubscriber = rospy.Subscriber(topic_name_Json, String, JsonTypeDataCallback)
    rospy.loginfo("...3")

    #rate = rospy.Rate(5)

    rospy.sleep(2) #delay time after starting the node
    rospy.loginfo("Waiting for Message...")

    rospy.set_param('Excute_Lock', False)
    #print rospy.get_param('Excute_Lock')

    rospy.spin()

        
    