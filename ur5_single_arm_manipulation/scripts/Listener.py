#!/usr/bin/env python
# encoding: utf-8
import sys
reload(sys)
sys.setdefaultencoding('utf-8')
import rospy
import json
from std_msgs.msg import String
import jsonFileTest
import ast

def callback(String):
    print("receiving data...")
    rospy.loginfo(rospy.get_caller_id() + 'I heard : \n%s', String.data)
  
    dict1  = json.loads(String.data)
    #print("p")
    json_tool_1 = jsonFileTest.String_Proccessor(dict1)

    comNum = json_tool_1.CommandSelection(0) #get the command code 
    rospy.loginfo("comNUm is %d", comNum)

    #rospy.set_param('Json_Data_Excute_Lock', False) # the Lock will not open untill ur5 fininshes the tasks


    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    topic_name = '/JsonData_Sender/JsonData'
    rospy.Subscriber(topic_name, String, callback)


    rospy.init_node('listener2', anonymous=True)

    topic_name2 = '/JsonData_Sender/JsonData2'
    rospy.Subscriber(topic_name2, String, callback)
    #ospy.set_param('JsonData_Excute_Lock', True)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

"""
def unicode_convert(input):
    if isinstance(input, dict):
        return {unicode_convert(key): unicode_convert(value) for key, value in input.iteritems()}
    elif isinstance(input, list):
        return [unicode_convert(element) for element in input]
    elif isinstance(input, unicode):
        return input.encode('utf-8')
    else:
        return input
"""

if __name__ == '__main__':
    listener()
