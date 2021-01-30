#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg

from moveit_commander import RobotCommander, PlanningSceneInterface
from moveit_commander import roscpp_initialize, roscpp_shutdown

from actionlib import SimpleActionClient, GoalStatus

from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Quaternion
from moveit_msgs.msg import PickupAction, PickupGoal, Constraints, OrientationConstraint
from moveit_msgs.msg import PlaceAction, PlaceGoal
from moveit_msgs.msg import PlaceLocation
from moveit_msgs.msg import MoveItErrorCodes
from moveit_simple_grasps.msg import GenerateGraspsAction, GenerateGraspsGoal, GraspGeneratorOptions

from tf.transformations import quaternion_from_euler
from ur5_single_arm_manipulation.msg import CosCommd

import sys
import copy
import numpy
import os

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)


# Create dict with human readable MoveIt! error codes:
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name


##############################################################################################################################
#########################################Position_INFO of (grasping object)########################################################


##############################################################################################################################
##############################################################################################################################
      
class Grasp_Place:
    def __init__(self):
        # Retrieve params:
        #if there is no param named 'table_object_name' then use the default
        self._table_object_name = rospy.get_param('~table_object_name', 'Grasp_Table')
        self._grasp_object_name = rospy.get_param('~grasp_object_name', 'Grasp_Object1')

        self._grasp_object_name_capscrew4 = rospy.get_param('~grasp_object_name4', 'capscrew4')  #capscrew
        self._grasp_object_name_screw = rospy.get_param('~grasp_object_name5', 'screw')  # screw
        #print  self._grasp_object_name_capscrew4



        self._arm_group     = rospy.get_param('~manipulator', 'manipulator')
        self._gripper_group = rospy.get_param('~gripper', 'gripper')


        # Create (debugging) publishers:
        self._grasps_pub = rospy.Publisher('grasps', PoseArray, queue_size=1, latch=True)
        self._places_pub = rospy.Publisher('places', PoseArray, queue_size=1, latch=True)

        # Create planning scene and robot commander:
        #self._scene = PlanningSceneInterface()
        self._robot = RobotCommander()

        rospy.sleep(1.0)

##############################################################################################################################
##############################################################################################################################
 
        #Retrieve groups (arm and gripper):
        self._arm     = self._robot.get_group(self._arm_group)
        self._gripper = self._robot.get_group(self._gripper_group)

        
        #get the pose of endfection
        print (self._arm.get_current_pose(self._arm.get_end_effector_link()).pose )

        
       
        #self._Init_robot_state()                                     #robot stata initilizations
        #self._Go_to_default_pose()                                   #set the robot pose to the 'home' pose set in moveit_setup_assistant (before a grasp)
        #self._move_to_obj_for_grasp(Grasp_pose_use, gripper_depth)   #move to the taget object for grasping
        #self._grasp_action(GRIPPER_CLOSED)                           #Close the gripper to grasp the object
        #self._move_up(displacementUp)                                #move up after the gripper grasp the target object
        #self._move_to_placeGoal(place_Pose_use, gripper_depth)       #move to the above area of the placeGoal pose(a little bit higher)
        #self._move_down(displacementDown)                            #move down to the placeGoal pose
        #self._place_action()                                         #place -> release or open the gripper
        #self._Go_to_default_pose()                                   #set the robot pose to the 'home' pose set in moveit_setup_assistant (after a place)
        

        #Costom_Command (Int)
        # 0:    _Init_robot_state() 
        # 1:    _Go_to_default_pose()  
        # 2:    _move_to_obj_for_grasp(Grasp_pose_use, gripper_depth) 
        # 3:    _grasp_action(GRIPPER_CLOSED) 
        # 4:    _move_up(displacementUp) 
        # 5:    _move_to_placeGoal(place_Pose_use, gripper_depth)  
        # 6:    _move_down(displacementDown)  
        # 7:    _place_action()

        ##########Receive Command and respongding data from the server###########






        #        _Go_to_default_pose()         #让机械臂到达默认位姿
        #        _move_to_obj_for_grasp(Grasp_pose_use, gripper_depth) # 移->到达螺母预抓取位姿态
        #        _grasp_action(GRIPPER_CLOSED) # 抓->控制爪子关闭
        #        _move_up(displacementUp)      # 移->向上提螺母
        #        _move_to_placeGoal(place_Pose_use, gripper_depth)     #移->趋向螺栓
        #        _move_down(displacementDown)  # 移-> 插入螺栓
        #        _place_action()               # 放->控制爪子放开

        


        self._Init_robot_state() 
  





    #Initialzing the state of the robot
    def _Init_robot_state(self):
 
        rospy.loginfo('Initializing the robot state')
        self._arm.set_goal_position_tolerance(0.001)
        self._arm.set_goal_orientation_tolerance(0.001)

        self._arm.allow_replanning(True)
  
        self._arm.set_pose_reference_frame(self._robot.get_planning_frame())

        self._arm.set_planning_time(7)


        

    #Set the 'home' pose of the ur5 before graspsing or after placing sth
    def _Go_to_default_pose(self):
        rospy.set_param('Excute_Lock', False)
        rospy.loginfo('Set the defaulit pose of the ur5 ')
        self._arm.set_named_target('home') #go to the 'home' place that are set in moveit_setup_assistant.
        self._arm.go()
        self._gripper.set_joint_value_target('gripper_finger1_joint',[0.0]) #[0.0] is the wide open pose of the gripper group. 
        self._gripper.go()



    #move to the object for grasp
    #gripper_depth : default->0.15
    def _move_to_obj_for_grasp(self, pose, gripper_depth):
        rospy.set_param('Excute_Lock', False)
        #Step-1##move-to-the-object-for-grasp###
        rospy.loginfo('moving-to-the-object-for-grasp')
        pose.position.z = pose.position.z + gripper_depth
        self._arm.set_pose_target(pose, self._arm.get_end_effector_link())
        traj = self._arm.plan()
        self._arm.execute(traj)
        #rospy.sleep(1)



    #grasp action
    #gripper_depth : default->0.15
    #GRIPPER_CLOSED is a list type and can be modified accodring to the object width
    def _grasp_action(self, GRIPPER_CLOSED):
        rospy.set_param('Excute_Lock', False)
        G_C = [GRIPPER_CLOSED]
        #Step-2##grasp###
        #grasp_clsoe posture()
        rospy.loginfo('grasping the object')
        self._gripper.set_joint_value_target('gripper_finger1_joint',G_C)
        self._gripper.go()
        #rospy.sleep(1)

   
    
    #move up 0.1   
    def _move_up(self, displacementUp):
        rospy.set_param('Excute_Lock', False)
        #Step-3##move-up after the grsap###
        rospy.loginfo('moving up')
        current_pose = self._arm.get_current_pose(self._arm.get_end_effector_link()).pose  #get the current pose of the end_effector

        current_pose.position.z = current_pose.position.z + displacementUp
        self._arm.set_pose_target(current_pose, self._arm.get_end_effector_link())
        traj = self._arm.plan()
        self._arm.execute(traj)
        #rospy.sleep(1)

    #move to the placeGoal
    def _move_to_placeGoal(self, pose, gripper_depth):
        rospy.set_param('Excute_Lock', False)
        #Step-4##move-to-the-placeGoal(a little bit higer in case of collision)###
        rospy.loginfo('move-to-the-placeGoal: a little bit higer in case of collision')
        pose.position.z = pose.position.z + gripper_depth
        self._arm.set_pose_target(pose,self._arm.get_end_effector_link())
        traj = self._arm.plan()
        self._arm.execute(traj)
        #rospy.sleep(1)


    #maybe->Step-5##move-down-to the placeGoal###
    def _move_down(self, displacementDown):
        rospy.set_param('Excute_Lock', False)
        #Step-5##move-down-to the placeGoal###
        rospy.loginfo('moving down')
        current_pose = self._arm.get_current_pose(self._arm.get_end_effector_link()).pose  #get the current pose of the end_effector

        current_pose.position.z = current_pose.position.z - displacementDown
        self._arm.set_pose_target(current_pose, self._arm.get_end_effector_link())
        traj = self._arm.plan()
        self._arm.execute(traj)
        #rospy.sleep(1)


    #maybe->Step-5##move-down-to the placeGoal###
    def _place_action(self):
        rospy.set_param('Excute_Lock', False)
        #Step-6##place###
        #grasp_clsoe posture()
        rospy.loginfo('opening the gripper and placing the object')
        self._gripper.set_joint_value_target('gripper_finger1_joint',[0.0])
        self._gripper.go()
        #rospy.sleep(1)

    ################### a switch case of if-else type function to cope with the command sent from the server ###############


def main():
    robot = Grasp_Place()
    
    

def topic_callback(msg, robot): #a case switch or if-else function

    #while (rospy.get_param('Excute_Lock')==True):
     #       rospy.loginfo('waiting to unlock the mutex ')
    rospy.loginfo('got the msg from server CommandNum is %d',msg.ComNum)
    rospy.loginfo('output the Pose_INFO')
    
    #rospy.loginfo('got the msg from server GRIPPER_Closed is %f',msg.GRIPPER_CLOSED)
    #rospy.loginfo('got the msg from server gripper_depth is %f',msg.gripper_depth)
    #rospy.loginfo('got the msg from server displacementUp is %f',msg.displacementUp)
    #rospy.loginfo('got the msg from server displacementDown is %f',msg.displacementDown)



    switch={
        0: case0,   
        1: case1,
        2: case2,
        3: case3,
        4: case4,
        5: case5,
        6: case6,
        7: case7
    }

    switch[msg.ComNum](msg, robot)

    rospy.set_param('Excute_Lock', True)

##############################################A way to decode the command sent from Server##########################################


        #Costom_Command (Int)
        # 0:    _Init_robot_state() 
        # 1:    _Go_to_default_pose()  
        # 2:    _move_to_obj_for_grasp(Grasp_pose_use, gripper_depth)
        # 3:    _grasp_action(GRIPPER_CLOSED) 
        # 4:    _move_up(displacementUp) 
        # 5:    _move_to_placeGoal(place_Pose_use, gripper_depth)  
        # 6:    _move_down(displacementDown)  
        # 7:    _place_action()

def case0(msg,robot):#reset
    robot._Init_robot_state()
    
def case1(msg,robot):
    robot._Go_to_default_pose()
    
def case2(msg,robot):
    robot._move_to_obj_for_grasp(msg.pose, msg.gripper_depth)   
    
def case3(msg,robot):
    robot._grasp_action(msg.GRIPPER_CLOSED) 
    
def case4(msg,robot):
    robot._move_up(msg.displacementUp) 
    
def case5(msg,robot):
    robot._move_to_placeGoal(msg.pose, msg.gripper_depth)  
    
def case6(msg,robot):
    robot._move_down(msg.displacementDown) 
    
def case7(msg,robot):
    robot._place_action()
    







if __name__ == '__main__':
    roscpp_initialize(sys.argv)
    rospy.init_node('atomic_Grasp_and_Place')


    robot = Grasp_Place()

    #main()
    topic_name_Command = '/Decider/CosCmd'
    sub = rospy.Subscriber(topic_name_Command, CosCommd, topic_callback, robot)
    
    rospy.loginfo("waiting for CosCmd...")
    rospy.set_param('Excute_Lock', True)
    rospy.sleep(2)

    rospy.spin()

    #roscpp_shutdown()