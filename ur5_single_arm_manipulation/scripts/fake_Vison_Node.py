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
from numpy import *
import os


#p1 stands for grasping pose
Grasp_pose_use= Pose()
#Grasp_pose_use.header.frame_id = '/world'
Grasp_pose_use.position.x = 0.86
Grasp_pose_use.position.y = 0
Grasp_pose_use.position.z = 0.445 #- grasp_z_err #- Base_Height# base_link as a reference frame
q_grasp = quaternion_from_euler(1.57, 1.57, 1.57) #Q number
Grasp_pose_use.orientation = Quaternion(*q_grasp)
#vector form data 
pos_Grasp = [0.86, 0.0, 0.445]  
angle_Grasp = [1.57, 1.57, 1.57]


# setting the placePose after grasping the stuff
Place_Z_OFFSET = 0.1 #if smaller than 0.03
Place_Y_OFFSET = 0.3

place_Pose_use = Pose()
place_Pose_use.position.x = 0.86
place_Pose_use.position.y = -0.30
place_Pose_use.position.z = 0.545  # place the stuff a little higher above the table (world axis as a reference)
q = quaternion_from_euler(1.57, 1.57, 0) #Q number
place_Pose_use.orientation = Quaternion(*q)
#vector form data 
pos_Place = [0.86, -0.30, 0.545]  
angle_Place = [1.57, 1.57, 0] 

###Constructing the PostList
PoseList = []
PoseList.append(pos_Grasp)
PoseList.append(angle_Grasp)
PoseList.append(pos_Place)
PoseList.append(angle_Place)

Object_array = random.rand()



def ObjectTypeDataCallback(Data): #A ptyhon str-type data is received.
        #coping with another situation
    String_indicator_1 = jsonFileTest.String_Proccessor(Data.data)
    rospy.loginfo("The Msg is in Call back.")
    rospy.loginfo(String_indicator_1.Datadict["response_code"])
    #if (String_indicator_1.Datadict["response_code"] == "200"): #code:201-> Object Name
    #Receiving the ObjectType Data to tell which object that the Vision Node need to detect
    rospy.loginfo("The Msg is Object entities Data.")

    ###################################################################################################
    # After detecting the Objects then send back 6D pose to the back-end node(Fill in the JsonType str
    ###################################################################################################
    Object_JsonStr = String_indicator_1._FillinTheStrWith6DPose(PoseList)
    print("get outta the method")
    rospy.loginfo(Object_JsonStr)

    ObjectPublisher.publish(Object_JsonStr)
    rospy.loginfo("published")
#else:
    #    pass
        


        
   



if __name__ == '__main__':
    # Set a node As a central Decider. 
    rospy.init_node('Fake_Vision_Node')


    # Set a topic that is responsible for receiving the JsonTypeStr that contain Objects
    topic_name_Object = '/Vision/JsonTypeData/entities_unprocessed'
    ObjectSubscriber = rospy.Subscriber(topic_name_Object, String, ObjectTypeDataCallback)
    rospy.loginfo("...1")


    # Set a topic that is responsible for receiving the JsonTypeStr that contain Objects
    topic_name_Object = '/Vision/JsonTypeData/entities_processed'
    ObjectPublisher = rospy.Publisher(topic_name_Object, String, queue_size=10)
    rospy.loginfo("...1")

    rospy.sleep(3) #delay for 3s.

    rospy.loginfo("Fake_Vision_Node is standby")
    rospy.spin()
