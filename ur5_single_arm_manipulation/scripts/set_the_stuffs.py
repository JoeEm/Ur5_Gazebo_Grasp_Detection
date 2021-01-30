#! /usr/bin/env python
# encoding: utf-8
# this file for pre-planning a task scene.

import rospy
import rospkg

from moveit_commander import RobotCommander, PlanningSceneInterface
from moveit_commander import roscpp_initialize, roscpp_shutdown
import moveit_commander

from actionlib import SimpleActionClient, GoalStatus

from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Quaternion
from moveit_msgs.msg import PickupAction, PickupGoal
from moveit_msgs.msg import PlaceAction, PlaceGoal
from moveit_msgs.msg import PlaceLocation
from moveit_msgs.msg import MoveItErrorCodes
from moveit_simple_grasps.msg import GenerateGraspsAction, GenerateGraspsGoal, GraspGeneratorOptions

from tf.transformations import quaternion_from_euler

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


# setting the global structure for stuffs like, table ,box ,or soemthin' else
table_x = 0.56 + 0.3
table_y = 0
table_z = 0.50  # increase z by 0.03 to make gripper reach block

box1_x = table_x
box1_y = table_y
# size of the box in its urdf is 0.045
#box1_z = 0.745+0.03
box1_z = table_z + 0.04


class planning:
    def __init__(self):
        # Retrieve params:
        # if there is no param named 'table_object_name' then use the default
        # set the scene in rzvi
        # original initilization is for 2
        self._table_object_name = rospy.get_param(
            '~table_object_name', 'Grasp_Table')  # table
        self._grasp_object_name_1 = rospy.get_param(
            '~grasp_object_name', 'Grasp_Object1')  # box1

        self._grasp_object_name_2 = rospy.get_param(
            '~grasp_object_name2', 'Wall_behind')  # wall_behind as a obstacle

        self._grasp_object_name_3 = rospy.get_param(
        '~grasp_object_name3', 'Wall_above')  # wall_above as a obstacle

        self._grasp_object_name_screw = rospy.get_param(
        '~grasp_object_name5', 'screw')  # screw
        #print  self._grasp_object_name_screw

        self._grasp_object_name_capscrew4 = rospy.get_param(
        '~grasp_object_name4', 'capscrew4')  # capscrew
        #print  self._grasp_object_name_capscrew4

        #### Not needed in the model spawn Section ####
        # Create (debugging) publishers(display in rviz):
        #self._grasps_pub = rospy.Publisher(
        #    'grasps', PoseArray, queue_size=1, latch=True)
        #self._places_pub = rospy.Publisher(
        #    'places', PoseArray, queue_size=1, latch=True)

        # Create planning scene and robot commander:
        self._scene = PlanningSceneInterface()
        self._robot = RobotCommander()

        rospy.sleep(1.0)

        # Clean the scene:
        self._scene.remove_world_object(self._table_object_name)
        self._scene.remove_world_object(self._grasp_object_name_1)
        self._scene.remove_world_object(self._grasp_object_name_2) #wall_behind
        #self._scene.remove_world_object(self._grasp_object_name_3) #wall_above
        self._scene.remove_world_object(self._grasp_object_name_capscrew4) #capscrew4
        self._scene.remove_world_object(self._grasp_object_name_screw) #screw
        



        ##############3# Add table and Coke can objects to the planning scene:#####################################
        self._pose_table = self._add_table(self._table_object_name)
        self._add_grasp_block_1(self._grasp_object_name_1)  

       

        

        # ##########################################3Adding obstacle_behind#############################
        p_wall = PoseStamped()
        robot = moveit_commander.RobotCommander()
        p_wall.header.frame_id = robot.get_planning_frame()
        p_wall.header.stamp = rospy.Time.now()

        p_wall.pose.position.x = -0.25
        p_wall.pose.position.y = 0.0
        p_wall.pose.position.z = 1.5
        q_wall_temp = quaternion_from_euler(0.0, 80.0, 0.0) #Q number
        p_wall.pose.orientation = Quaternion(*q_wall_temp)

        self._scene.add_box(self._grasp_object_name_2, p_wall, (3, 3, 0.05))    

        ###################################### Adding obstacle wall_above######################################
        p_wall2 = PoseStamped()
        p_wall2.header.frame_id = robot.get_planning_frame()
        p_wall2.header.stamp = rospy.Time.now()

        p_wall2.pose.position.x = table_x
        p_wall2.pose.position.y = table_y
        p_wall2.pose.position.z = 1.25
        q_wall2_temp = quaternion_from_euler(0.0, 0.0, 0.0) #Q number
        p_wall2.pose.orientation = Quaternion(*q_wall2_temp)

        #self._scene.add_box(self._grasp_object_name_3, p_wall2, (3, 3, 0.05))   

        ###################################### Adding obstacle side-1#####################################
        p_wall3 = PoseStamped()
        p_wall3.header.frame_id = robot.get_planning_frame()
        p_wall3.header.stamp = rospy.Time.now()

        p_wall3.pose.position.x = table_x
        p_wall3.pose.position.y = table_y - 0.8/2 - 0.2
        p_wall3.pose.position.z = table_z
        q_wall3_temp = quaternion_from_euler(80.0, 0.0, 0.0) #Q number
        p_wall3.pose.orientation = Quaternion(*q_wall3_temp)

        self._scene.add_box("wall_side-1", p_wall3, (2, 2, 0.05))   


         ###################################### Adding obstacle side -2######################################
        p_wall4 = PoseStamped()
        p_wall4.header.frame_id = robot.get_planning_frame()
        p_wall4.header.stamp = rospy.Time.now()

        p_wall4.pose.position.x = table_x
        p_wall4.pose.position.y = table_y + 0.8/2 + 0.05
        p_wall4.pose.position.z = table_z
        q_wall4_temp = quaternion_from_euler(80.0, 0.0, 0.0) #Q number
        p_wall4.pose.orientation = Quaternion(*q_wall4_temp)

        self._scene.add_box("wall_side-2", p_wall4, (2, 2, 0.05)) 


        ##############################################adding screw###########################
        screw = PoseStamped()
        screw.header.frame_id = robot.get_planning_frame()
        screw.header.stamp = rospy.Time.now()

        screw.pose.position.x = table_x
        screw.pose.position.y = table_y - 0.2
        screw.pose.position.z = box1_z + 0.02
        screw_q = quaternion_from_euler(0.0, 0.0, 0.0) #Q number
        screw.pose.orientation = Quaternion(*screw_q)

        print self._scene.add_box(self._grasp_object_name_screw, screw, (0.036, 0.046, 0.032)) 




        ##############################################adding capscrew 0.045###########################
        capscrew4 = PoseStamped()
        capscrew4.header.frame_id = robot.get_planning_frame()
        capscrew4.header.stamp = rospy.Time.now()

        capscrew4.pose.position.x = table_x 
        capscrew4.pose.position.y = table_y 
        capscrew4.pose.position.z = box1_z + 0.02
        q4 = quaternion_from_euler(0.0, 0.0, 0.0) #Q number
        capscrew4.pose.orientation = Quaternion(*q4)

        print self._scene.add_box(self._grasp_object_name_capscrew4, capscrew4, (0.05, 0.05, 0.040)) 



        rospy.sleep(1.0)
        # gripper graspping point setting.
        # Define target place pose:

#########################################Section: Neccessay function for (displaying)Rviz###################################
######this section for adding stuffs in the planning scene.(table, box1, box2, etc...) in RViz############################3
    def _add_table(self, name):
        """
        Create and add table to the scene
        """
        p = PoseStamped()
        p.header.frame_id = self._robot.get_planning_frame()
        p.header.stamp = rospy.Time.now()

        p.pose.position.x = table_x
        p.pose.position.y = table_y
        p.pose.position.z = table_z

        q = quaternion_from_euler(0.0, 0.0, numpy.deg2rad(90.0))
        p.pose.orientation = Quaternion(*q)

        # Table size from ~/.gazebo/models/table/model.sdf, using the values
        # for the surface link.
        # display in RZvi
        print self._scene.add_box(name, p, (1, 1, 0.05)) #arg(name, pose, size)

        return p.pose

    def _add_grasp_block_1(self, name):
        """
        Create and add block to the scene
        """
        p = PoseStamped()
        p.header.frame_id = self._robot.get_planning_frame()
        p.header.stamp = rospy.Time.now()

        p.pose.position.x = box1_x + 0.3
        p.pose.position.y = box1_y + 0.3
        p.pose.position.z = box1_z

        q = quaternion_from_euler(0.0, 0.0, 0.0)
        p.pose.orientation = Quaternion(*q)

    # Coke can size from ~/.gazebo/models/coke_can/meshes/coke_can.dae,
    # using the measure tape tool from meshlab.
    # The box is the bounding box of the coke cylinder.
    # The values are taken from the cylinder base diameter and height.
        #self._scene.add_box(name, p, (0.045, 0.045, 0.045)) 

        return p.pose

##############################################################################################################
##############################################################################################################
def main():
    p = planning()

    rospy.spin()

############################################################################################################
#########################################Section: Neccessay function for Gazebo###################################
def delete_gazebo_model(models):

    # Delete model in gazebo

    try:
        delete_model = rospy.ServiceProxy(
            '/gazebo/delete_model', DeleteModel)
        for a_model in models:
            resp_delete = delete_model(a_model)
    except rospy.ServiceException, e:
        rospy.loginfo("Delete Model service call failed: {0}".format(e))

def spawn_gazebo_model(model_path, model_name, model_pose, reference_frame="world"):
    # Spawn model in gazebo
    model_xml = ''
    with open(model_path, "r") as model_file:
        model_xml = model_file.read().replace('\n', '')
        rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy(
            '/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf(model_name, model_xml,
                                "/", model_pose, reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))

###################################################################################################


if __name__ == '__main__':
    roscpp_initialize(sys.argv)
    rospy.init_node('planning')

    rospack = rospkg.RosPack()
    pack_path = rospack.get_path('ur5_single_arm_tufts')
    """
    rosrun gazebo_ros spawn_model -file $(rospack find ur5_single_arm_tufts)/urdf/objects/table.urdf -urdf -x 0.85 -y 0.0 -z 0.73 -model my_object
    rosrun gazebo_ros spawn_model -file $(rospack find ur5_single_arm_tufts)/urdf/objects/block.urdf -urdf -x 0.5 -y -0.0 -z 0.77 -model block
    """
    # add the urdf file into the world
    table_path = pack_path+os.sep+'urdf'+os.sep+'objects'+os.sep+'table.urdf'
    table_name = 'table'
    # increase z by 0.03 to make gripper reach block
    table_pose = Pose(position=Point(x=table_x, y=table_y, z=table_z))

    block1_path = pack_path+os.sep+'urdf'+os.sep+'objects'+os.sep+'block.urdf'
    block1_name = 'block1'
    block1_pose = Pose(position=Point(x=box1_x, y=box1_y, z=box1_z))


    ###1 screw with 3 different sizes of  cap screw
    screw_path = pack_path+os.sep+'urdf'+os.sep+'objects'+os.sep+'single_screw.urdf'
    screw_name = 'screw'
    screw_pose = Pose(position=Point(x=box1_x, y=box1_y-0.2, z=box1_z+0.01))

    #capscrew :scale 0.03
    capscrew_path = pack_path+os.sep+'urdf'+os.sep+'objects'+os.sep+'capscrew.urdf'
    capscrew_name = 'capscrew'
    capscrew_pose = Pose(position=Point(x=box1_x+0.2, y=box1_y+0.2, z=box1_z - 0.01))

    #capscrew :scale  0.04
    capscrew_size2_path = pack_path+os.sep+'urdf'+os.sep+'objects'+os.sep+'capscrew_size2.urdf'
    capscrew_size2_name = 'capscrew_size2'
    capscrew_size2_pose = Pose(position=Point(x=box1_x+0.2, y=box1_y, z=box1_z - 0.01))

    #capscrew :scale  0.035
    capscrew_size3_path = pack_path+os.sep+'urdf'+os.sep+'objects'+os.sep+'capscrew_size3.urdf'
    capscrew_size3_name = 'capscrew_size3'
    capscrew_size3_pose = Pose(position=Point(x=box1_x+0.2, y=box1_y+0.1, z=box1_z-0.01))

    #capscrew :scale  0.045
    capscrew_size4_path = pack_path+os.sep+'urdf'+os.sep+'objects'+os.sep+'capscrew_size4.urdf'
    capscrew_size4_name = 'capscrew_size4'
    capscrew_size4_pose = Pose(position=Point(x=box1_x, y=box1_y, z=box1_z + 0.04))

    #capscrew :scale  0.045 [FOR KINECT Test!!!!!!!!!!!!!!!!!!!!!]
    capscrew_size5_path = pack_path+os.sep+'urdf'+os.sep+'objects'+os.sep+'capscrew_size4.urdf'
    capscrew_size5_name = 'capscrew_size_for_kinect_test'
    capscrew_size5_pose = Pose(position=Point(x=2.5, y=0.2, z=0))

    #capscrew_plugin: scale


    #Wall behind
    wall_path = pack_path+os.sep+'urdf'+os.sep+'objects'+os.sep+'wall.urdf'
    wall_name = 'Wall'
    wall_pose= Pose()
    wall_pose.position.x = -0.25  
    wall_pose.position.y = 0
    wall_pose.position.z = 1 #base_link as a reference frame
    q_wall = quaternion_from_euler(0.0, 80.0, 0.0) #Q number
    wall_pose.orientation = Quaternion(*q_wall)  

    # add wall from above
    ceilling_path = pack_path+os.sep+'urdf'+os.sep+'objects'+os.sep+'table.urdf'
    ceilling_name = 'ceiling'
    # increase z by 0.03 to make gripper reach bloc0.7
    ceilling_pose = Pose(position=Point(x=table_x, y=table_y, z=table_z + 1))
  

    delete_gazebo_model([table_name, block1_name, screw_name, capscrew_name, capscrew_size2_name, capscrew_size3_name, capscrew_size4_name])
    #delete_gazebo_model([ceilling_name,wall_name])
    delete_gazebo_model([capscrew_size5_name])
    spawn_gazebo_model(capscrew_size5_path, capscrew_size5_name, capscrew_size5_pose)


    spawn_gazebo_model(table_path, table_name, table_pose)
    #spawn_gazebo_model(block1_path, block1_name, block1_pose)

    spawn_gazebo_model(screw_path, screw_name, screw_pose)

    #spawn_gazebo_model(capscrew_path, capscrew_name, capscrew_pose)
    #spawn_gazebo_model(capscrew_size2_path, capscrew_size2_name, capscrew_size2_pose)
    #spawn_gazebo_model(capscrew_size3_path, capscrew_size3_name, capscrew_size3_pose)
    spawn_gazebo_model(capscrew_size4_path, capscrew_size4_name, capscrew_size4_pose)

    #spawn_gazebo_model(wall_path, wall_name, wall_pose)
    #spawn_gazebo_model(ceilling_path, ceilling_name,ceilling_pose)



    
    main()

    roscpp_shutdown()