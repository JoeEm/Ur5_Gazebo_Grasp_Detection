#!/usr/bin/env python3
# encoding: utf-8
import rospy
import json
from std_msgs.msg import String
  

#############################################JsonType Data For Testing########################################

data = '''
{
    "response_code": "200",
    "MaxNumber" : "7",
    "data":{
        "sequences":[
            {
                "skills":"init",
                "primitives":"回",
                "target_lable":"",
                "target_pose":"N",
                "target_angle":"N",
                "target_high":"N"
            },
            {
                "skills":"抓取",
                "primitives":"趋",
                "target_lable":"螺母",
                "target_pose":"(0.86,0,0.545)",
                "target_angle":"(1.57, 1.57, 0)",
                "target_high":"N"
            },
            {
                "skills":"抓取",
                "primitives":"抓",
                "target_lable":"螺母",
                "target_pose":"N",
                "target_angle":"N",
                "target_high":"N"
            },
            {
                "skills":"抓取",
                "primitives":"提",
                "target_lable":"螺母",
                "target_pose":"N",
                "target_angle":"N",
                "target_high":"N"
            },
            {
                "skills":"对准",
                "primitives":"移",
                "target_lable":"螺栓",
                "target_pose":"(0.86,-0.20,0.645)",
                "target_angle":"(1.57, 1.57, 1.57)",
                "target_high":"N"
            },
            {
                "skills":"对准",
                "primitives":"下",
                "target_lable":"螺栓",
                "target_pose":"N",
                "target_angle":"N",
                "target_high":"N"
            },
            {
                "skills":"插入",
                "primitives":"放",
                "target_lable":"螺母",
                "target_pose":"N",
                "target_angle":"N",
                "target_high":"N"
            }
        ]
    }
}
'''

##############################################################################################################

#in_json = json.dumps(data) # Encode the data  From json to str
#Datadict = json.loads(self.in_json) # Decode into a Python object(dictionary)
#MaxNumOfSequecnces = int(self.Datadict["MaxNumber"])
#print("data format is:...")
#rospy.loginfo("%s", in_json)

#dict1 = json.loads(data)



if __name__ == '__main__':

    rospy.init_node('Backend')
    rospy.set_param('JsonData_Excute_Lock', False)

    topic_name = '/Decider/JsonTypeData/Sequences'

    publisher = rospy.Publisher(topic_name, String, queue_size=10)

    rospy.sleep(3) # pause for a while

    #while (rospy.get_param('JsonData_Excute_Lock')==False):
    #    rospy.loginfo('waiting to unlock the JsonData_mutex ')
    #while( rospy.set_param('JsonData_Excute_Lock', False))
    
    rospy.loginfo("get outta the while loop")
    publisher.publish(data)
    rospy.loginfo("publish the data!")
    
    # rsopy.spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

