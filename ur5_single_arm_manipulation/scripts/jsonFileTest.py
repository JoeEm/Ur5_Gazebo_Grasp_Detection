#! /usr/bin/env python
# -*- coding: utf-8 -*-
import sys
reload(sys)
sys.setdefaultencoding('utf-8')
import json
from geometry_msgs.msg import Pose,Quaternion
from tf.transformations import quaternion_from_euler
from numpy import *




class String_Proccessor:
    def __init__(self, Data): #Data is a JsonType Str.
        #self.in_json = json.dumps(Data) # Encode the data  From json to str
        #self.Datadict = json.loads(self.in_json) # Decode into a Python object(dictionary)
        self.Datadict = json.loads(Data) #convert the str to python dict    

    def _getMaxNum_Of_Sequences(self):

        MaxNumOfSequecnces = int(self.Datadict["MaxNumber"])
        return MaxNumOfSequecnces

    def _getMaxNum_Of_Object(self):

        MaxNumOfObject = len(self.Datadict["param_entity"])
        return MaxNumOfObject

    def _check_Pose_is_needed(self, number): #if target_angle:"0" False
        if ( str(self.Datadict["data"]["sequences"][number]["target_angle"]) == "N"):
            return False
        else:
            return True

    def CommandSelection(self, number):
        if ( str(self.Datadict["data"]["sequences"][number]["primitives"]) == "回"):
            return 1

        if ( str(self.Datadict["data"]["sequences"][number]["primitives"]) == "趋"):
            return 2

        if ( str(self.Datadict["data"]["sequences"][number]["primitives"]) == "抓"):
            return 3
        
        if ( str(self.Datadict["data"]["sequences"][number]["primitives"]) == "提"):
            return 4
    
        if ( str(self.Datadict["data"]["sequences"][number]["primitives"]) == "移"):
            return 5

        if ( str(self.Datadict["data"]["sequences"][number]["primitives"]) == "下"):
            return 6
        
        if ( str(self.Datadict["data"]["sequences"][number]["primitives"]) == "放"):
            return 7
        

    def _str2NumForPose(self, number): #to process the str in the form of "(x,y,z)"
        Pose_tmp = Pose()

        ###### Processing the 3D position pose######
        str3D    = self.Datadict["data"]["sequences"][number]["target_pose"]
        strEuler = self.Datadict["data"]["sequences"][number]["target_angle"]

        result1 = str3D.replace('(', '')
        #print(result1)
        result2 = result1.replace(')', '')
        #print(result2)
        result3 = result2.split(',')
        #print(result3)

        Pose_tmp.position.x = float(result3[0])
        #print(Pose_tmp.position.x)
        Pose_tmp.position.y = float(result3[1])
        #print(Pose_tmp.position.y)
        Pose_tmp.position.z = float(result3[2])
        #print(Pose_tmp.position.z)
        #print(Pose_tmp)

        ###### Processing the orientation######

        result1 = strEuler.replace('(', '')
        #print(result1)
        result2 = result1.replace(')', '')
        #print(result2)
        result3 = result2.split(',')
        #print(result3)

        a = float(result3[0])
        b = float(result3[1])
        c = float(result3[2])
        q = quaternion_from_euler(a,b,c)
        Pose_tmp.orientation = Quaternion(*q)

        return Pose_tmp

    def _FillinTheStrWith6DPose(self, PoseList): #6D_Pose: compliant to Pose() class data, and is a list type!                                    #PoseArray size: (2*NumberOfObject ,3) <-> (rows, cols)
        
        print ("In the _FillinTheStrWith6DPose function")
        NumberOfObject = self._getMaxNum_Of_Object()
        PoseMatrix = random.rand(2*NumberOfObject, 3) #pos1->angle1->pos2->angle2
        print (PoseMatrix.shape[0])
        print (PoseMatrix.shape[1])
        BackUpDict = self.Datadict

        for row in range(0, PoseMatrix.shape[0]):
            for col in range(0, PoseMatrix.shape[1]):
                if (row < 2):
                    print ("processing")
                    if (col == 0):
                        a_tmp = str(BackUpDict["param_entity"][row/2]["target_pose"])
                        b_tmp = a_tmp.replace('x', str(PoseList[row][col]))
                        BackUpDict["param_entity"][row/2]["target_pose"] = b_tmp
                    if (col == 1):
                        a_tmp =  str(BackUpDict["param_entity"][row/2]["target_pose"])
                        b_tmp = a_tmp.replace('y', str(PoseList[row][col]))
                        BackUpDict["param_entity"][row/2]["target_pose"] = b_tmp
                    if (col == 2): 
                        a_tmp = str(BackUpDict["param_entity"][row/2]["target_pose"])
                        b_tmp = a_tmp.replace('z', str(PoseList[row][col])) 
                        BackUpDict["param_entity"][row/2]["target_pose"] = b_tmp
                else:
                    print ("processing")
                    if (col == 0):
                        a_tmp = str(BackUpDict["param_entity"][row/2]["target_angle"])
                        b_tmp = a_tmp.replace('x', str(PoseList[row][col]))
                        BackUpDict["param_entity"][row/2]["target_angle"] = b_tmp
                    if (col == 1):
                        a_tmp =  str(BackUpDict["param_entity"][row/2]["target_angle"])
                        b_tmp = a_tmp.replace('y', str(PoseList[row][col]))
                        BackUpDict["param_entity"][row/2]["target_angle"] = b_tmp
                    if (col == 2): 
                        a_tmp = str(BackUpDict["param_entity"][row/2]["target_angle"])
                        b_tmp = a_tmp.replace('z', str(PoseList[row][col])) 
                        BackUpDict["param_entity"][row/2]["target_angle"] = b_tmp


        #self.Datadict = json.loads(Data)
        print ("get outta the for loop!")
        BackUpStr = json.dumps(BackUpDict)



        return BackUpStr


if __name__ == '__main__':
    #print("creating the instance")
    #String_tool = String_Proccessor(data)
    #print("finishing the processing")
    #pose_tmp = (String_tool.Str2NumForPose())
    #print(result)
    pass