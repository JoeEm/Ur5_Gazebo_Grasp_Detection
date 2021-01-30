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
            "target_angle":"θ"
        },
        {
            "target_lable":"螺栓",
            "target_pose":"(x,y,z)",
            "target_angle":"θ"
        }
    ]
}
'''


def ObjectCallback(Data):
    #create a instance for processing the JosonType str
    String_indicator = jsonFileTest.String_Proccessor(Data.data)
    
    



if __name__ == '__main__':
    # Set a node As a central Decider. 
    rospy.init_node('Pose_Processor')


    topic_name_Command = '/Decider/CosCmd'
    ComandPublisher = rospy.Publisher(topic_name_Command, String, queue_size=10)
    rospy.loginfo("...1")
    
    topic_name_Get_object = 'param_entities'
    JsonSubscriber = rospy.Subscriber(topic_name_Get_object, String, ObjectCallback)
    rospy.loginfo("...3")

    #rate = rospy.Rate(5)

    rospy.sleep(2) #delay time after starting the node
    rospy.loginfo("Waiting for Message...")


    rospy.spin()
