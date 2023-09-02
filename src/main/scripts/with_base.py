#!/usr/bin/env python3

# UR5 arms path planning for human gesture detecting with YOLO
# Author : Charlotte Wang

import os
import sys
import time
import copy
import math
import rospy
import random
import struct
import numpy as np
import moveit_msgs.msg
import moveit_commander
import geometry_msgs.msg

from moveit_msgs.msg import Grasp
from grasping_msgs.msg import FindGraspableObjectsResult
from std_msgs.msg import Float32MultiArray, String, Bool, Float32
from sensor_msgs.msg import PointCloud2, JointState
from colorama import Fore, Back, Style
from moveit_msgs.msg import Constraints, JointConstraint, MoveGroupActionFeedback
from tf import transformations as trans


os.environ['ROS_MASTER_URI'] = 'http://192.168.1.148:11311'

def quaternion_rotation_matrix(Q):
    """
    Covert a quaternion into a full three-dimensional rotation matrix.
 
    Input
    :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
 
    Output
    :return: A 3x3 element matrix representing the full 3D rotation matrix. 
             This rotation matrix converts a point in the local reference 
             frame to a point in the global reference frame.
    """
    # Extract the values from Q
    q0 = Q[0]
    q1 = Q[1]
    q2 = Q[2]
    q3 = Q[3]
     
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
                            
    return rot_matrix

def human_cam_trans(input:np.array):
    if input[2] > 0.9:
            print("adjust depth value")
            input[2] = 0.9
    input = [input[2], -input[0], -input[1]]
    # x offset = -0.18m, y offset = 0.20m, z offset = 0.325m
    offset = np.array([0.0, -0.2, 0.265])
    output = np.add(offset, input)

    print("New coordinate respect to base:\n", output)
    return output

def tool_replace(tool):
    if "Spanner" == tool:
        return "Wrench"
    elif "Allen" == tool:
        return "Screwdriver"
    if "Spanner" == tool:
        return "Adjustable Wrench"
    elif "Allen" == tool:
        return "Screwdriver Bit Set"
    else:
        return tool

class Hand:
    def __init__(self, right_x=[], right_y=[], right_z=[], left_x=[], left_y=[], left_z=[]):
        self.right_x = right_x
        self.right_y = right_y
        self.right_z = right_z
        self.left_x = left_x
        self.left_y = left_y
        self.left_z = left_z

class TargetObject():
    def __init__(self, pose=None, dimension=None):
        self.pose = geometry_msgs.msg.PoseStamped()
        self.dimension = [0.0, 0.0, 0.0]

class arm(object) :
    def __init__(self):
        rospy.init_node('arm_node')
        super(arm, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)        

        # RobotCommander: Provides robot's kinematic model and current joint states
        robot = moveit_commander.RobotCommander()

        # PlanningSceneInterface: Provides remote interface for getting, setting, and updating the robot's internal understanding of the surrounding
        scene = moveit_commander.PlanningSceneInterface()

        # MoveGroupCommander: for planning group (group of joints)
        self.robot_traj_status = False        
        move_group = moveit_commander.MoveGroupCommander("manipulator")
        eef_group = moveit_commander.MoveGroupCommander("gripper")
        self.target_tool = ""
        self.hand = Hand()
        self.human = None
        self.object = None
        self.tool_detect_completed = False
        self.find_result = None 
        self.tool_axis = None
        self.move_group_feedback = ""
        self.finger_joint = 0.0
        
        planning_frame = move_group.get_planning_frame()
        eef_link = move_group.get_end_effector_link()
        group_names = robot.get_group_names()

        # region Grasp message generation 
        grasps = Grasp()
        grasps.pre_grasp_posture.joint_names = ['finger_joint']
        grasps.grasp_posture.joint_names = ['finger_joint']
        grasps.grasp_quality = 0.25
        grasps.pre_grasp_posture.points = [0.0]
        grasps.grasp_posture.points = [0.8]
        grasps.pre_grasp_approach.direction.vector = [0, 0, 1]
        grasps.pre_grasp_approach.desired_distance = 0.12
        grasps.pre_grasp_approach.min_distance = 0.0
        grasps.post_grasp_retreat.direction.vector = [0, 0, -1]
        grasps.post_grasp_retreat.desired_distance = 0.08
        grasps.post_grasp_retreat.min_distance = 0.05
        grasps.pre_grasp_approach.direction.header.frame_id = 'wrist_3_link'
        grasps.post_grasp_retreat.direction.header.frame_id = 'wrist_3_link'
        # endregion

        # region Initilizing
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.move_group.set_planning_pipeline_id("pilz_industrial_motion_planner")
        self.move_group.set_planner_id('LIN')
        self.move_group.set_planning_time(5.0)
        self.eef_group = eef_group
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        self.grasps = grasps
        self.scene.remove_world_object()
        self.table_empty = False
        # endregion

        # region Constrains
        constraints = Constraints()
        
        # elbow
        elbow_joint_constrain = JointConstraint()
        elbow_joint_constrain.joint_name = 'elbow_joint'
        elbow_joint_constrain.position = 0.0
        elbow_joint_constrain.tolerance_below = 15 # unit: degree
        elbow_joint_constrain.tolerance_above = 135
        elbow_joint_constrain.weight = 0.7
        constraints.joint_constraints.append(elbow_joint_constrain)
        
        # shoulder
        shoulder_lift_joint_constrain = JointConstraint()
        shoulder_lift_joint_constrain.joint_name = 'shoulder_lift_joint'
        shoulder_lift_joint_constrain.position = 0.0
        shoulder_lift_joint_constrain.tolerance_above = -10
        constraints.joint_constraints.append(shoulder_lift_joint_constrain)

        # base
        shoulder_pan_joint = JointConstraint()
        shoulder_pan_joint.joint_name = 'shoulder_pan_joint'
        shoulder_pan_joint.position = 0.0
        shoulder_pan_joint.tolerance_above = 30
        shoulder_pan_joint.tolerance_below = -120 
        constraints.joint_constraints.append(shoulder_pan_joint)

        move_group.set_path_constraints(constraints)
        # endregion
        
        # region Subs and pubs:
        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        self.move_group_feedback_sub = rospy.Subscriber(
            '/move_group/feedback',
            MoveGroupActionFeedback,
            self.move_group_feedback_cb
        )

        self.joint_states_sub = rospy.Subscriber(
            '/joint_states',
            JointState,
            self.joint_states_cb
        )

        #region Tool detection
        self.target_tool_pub = rospy.Publisher(
            '~target_tool',
            String,
            queue_size=1
        )        
        self.graspable_result_sub = rospy.Subscriber(
            '/tool_detect_node/graspable_tool',
            FindGraspableObjectsResult,
            self.graspable_tool_cb
        )
        self.tool_detect_completed_sub = rospy.Subscriber(
            '/tool_detect_node/completed',
            Bool, 
            self.tool_detect_completed_cb
        )
        self.tool_detect_completed_sub = rospy.Subscriber(
            '/tool_detect_node/tool_axis',
            String, 
            self.tool_axis_cb
        )
        #endregion   

        #region Human and hand detection
        self.human_hand_detect_pub = rospy.Publisher(
            '~human_hand_detect',
            Float32,
            queue_size=1
        )
        self.human_sub = rospy.Subscriber(
            '/human_hand_node/human_info', 
            Float32MultiArray, 
            self.human_sub_cb
        )
        self.hand_sub = rospy.Subscriber(
            '/human_hand_node/hand_info', 
            PointCloud2, 
            self.hand_sub_cb
        )
        self.human_hand_completed_sub = rospy.Subscriber(
            '/human_hand_node/completed',
            Bool, 
            self.human_hand_completed_cb
        )
        self.human_hand_completed = False
        #endregion
            
        #region Master main script
        self.pickup_sub = rospy.Subscriber(
            '/main_node/pickup_tool', 
            String,             
            self.pickup_cb
        )
        self.pickup_feedback_pub = rospy.Publisher(
            '/arm_node/pickup_feedback',
            String,
            queue_size=1
        )
        self.handoff_sub = rospy.Subscriber(
            '/main_node/handoff_signal', 
            String, 
            self.handoff_cb
        )
        self.handoff_feedback_pub = rospy.Publisher(
            '/arm_node/handoff_feedback',
            String,
            queue_size=1
        )
        self.reset_sub = rospy.Subscriber(
            '/main_node/reset',
            String,
            self.reset_cb
        )
        self.reset_feedback_pub = rospy.Publisher(
            '/arm_node/reset_feedback',
            String,
            queue_size=1
        )
        #endregion
        
        # endregion
        
        print("\nCurrent Pose:\n", move_group.get_current_joint_values())
        print(move_group.get_current_pose().pose)
        rospy.loginfo("Ready for accept commands from Master")
    
    # region Gripper
    def pre_grasp_posture(self): # Open gripper
        eef_group = self.eef_group
        grasps = self.grasps 
        gripper_pose = eef_group.get_current_joint_values()
        gripper_pose[0] = grasps.pre_grasp_posture.points[0] # Open finger
        eef_group.go(gripper_pose, wait=True)
        eef_group.stop() # Ensures that there is no residual movement
    
    def grasp_posture(self): # Close gripper         
        eef_group = self.eef_group
        grasps = self.grasps
        gripper_pose = eef_group.get_current_joint_values()
        gripper_pose[0] = grasps.grasp_posture.points[0] # Close finger
        eef_group.go(gripper_pose, wait=True)
        eef_group.stop()
    # endregion
    
    # region Pose
    def starting_pose(self) -> str:
        move_group = self.move_group
        move_group.set_max_velocity_scaling_factor(0.3)
        move_group.set_max_acceleration_scaling_factor(0.3)

        joint_goal = [0.008436894975602627, -2.2472408453570765, 1.9655742645263672, -1.2976849714862269, -1.5677931944476526, -0.011224095021383107]
        move_group.go(joint_goal, wait=True)
        rospy.sleep(0.1)
        move_group.stop()
        rospy.sleep(0.1)
        return self.move_group_feedback
    
    def tool_inhand_pose(self):
        move_group = self.move_group
        move_group.set_max_velocity_scaling_factor(0.3)
        move_group.set_max_acceleration_scaling_factor(0.3)

        joint_goal = [0.017220957204699516, -2.3268540541278284, 2.260349750518799, -3.0598705450641077, -1.585116211568014, -0.019635979329244435]
        move_group.go(joint_goal, wait=True)
        rospy.sleep(0.1)
        move_group.stop()
        rospy.sleep(0.1)
        return self.move_group_feedback
    
    def scaning_pose(self, pose='center') -> str:
        move_group = self.move_group
        move_group.set_max_velocity_scaling_factor(0.3)
        move_group.set_max_acceleration_scaling_factor(0.3)

        # Three kind of scanning pose
        if pose == 'left':
            joint_goal = [0.2618013918399811, -1.633786980305807, 1.3009371757507324, -1.2532065550433558, -1.567134205495016, 0.2997041344642639]
        elif pose == 'center':
            joint_goal = [-0.1874778906451624, -1.6312454382525843, 1.2965798377990723, -1.2432053724872034, -1.5650742689715784, -0.20414525667299444]
        elif pose == 'right':
            joint_goal = [-0.6572168509112757, -1.6313055197345179, 1.2966632843017578, -1.243145767842428, -1.5650861899005335, -0.6690128485309046]
        
        move_group.go(joint_goal, wait=True)
        rospy.sleep(0.1)
        move_group.stop()
        rospy.sleep(0.1)
        return self.move_group_feedback
    
    def video(self, pose='1') -> str:
        move_group = self.move_group
        move_group.set_max_velocity_scaling_factor(0.3)
        move_group.set_max_acceleration_scaling_factor(0.3)

        # Pose for video shooting
        if pose == '3':
            joint_goal = [0.048240624368190765, -2.3475969473468226, 1.9380087852478027, -1.1951969305621546, -1.6112335363971155, 0.06516909599304199]
        elif pose == '2':
            joint_goal = [0.21649952232837677, -1.2791245619403284, 1.680013656616211, -2.0345032850848597, -1.5804560820208948, 0.06461632251739502]
        elif pose == '1':
            joint_goal = [0.21874994039535522, -1.2387960592852991, 1.6947407722473145, -2.0814908186541956, -1.5803006331073206, 0.0652170330286026]
        elif pose == '4':
            joint_goal = [2.8699519634246826, -1.7153599897967737, 1.826087474822998, -1.6912091414081019, -1.5389702955829065, 1.3687787055969238]
        elif pose == '5':
            joint_goal = [2.870323896408081, -1.4394033590899866, 2.0979480743408203, -2.126458470021383, -1.5388744513141077, 1.3678555488586426]
        elif pose =='3.5':
            joint_goal = [1.4515982866287231, -2.166178051625387, 1.9284172058105469, -1.1924813429461878, -1.61082631746401, 0.06518107652664185]
        elif pose == '6':
            joint_goal = [3.0265703201293945, -1.4348128477679651, 2.1991591453552246, -2.3506959120379847, -1.668431584035055, 1.4348251819610596]

        move_group.go(joint_goal, wait=True)
        rospy.sleep(0.1)
        move_group.stop()
        rospy.sleep(0.1)
        return self.move_group_feedback
    
    #endregion

    # region Display and Execute plan
    def display_trajectory(self, plan):
        robot = self.robot

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        rate = rospy.Rate(1)
        
        self.display_trajectory_publisher.publish(display_trajectory)
        rate.sleep()

    def execute_plan(self, plan) -> str:
        move_group = self.move_group
        move_group.execute(plan, wait=True)

        return self.move_group_feedback
    # endregion

    # region Subscriber
    def hand_to_info(self, num):
        string = '\nRight hand:[' + str(self.hand.right_x[num]) + ', ' + str(self.hand.right_y[num]) + ', ' + str(self.hand.right_z[num]) + ']'
        string += '\nLeft hand:[' + str(self.hand.left_x[num]) + ', ' + str(self.hand.left_y[num]) + ', ' + str(self.hand.left_z[num]) + ']'
        return string

    def hand_sub_cb(self, sub):
        self.hand.right_x = [0.0] * 21
        self.hand.right_y = [0.0] * 21
        self.hand.right_z = [0.0] * 21
        self.hand.left_x = [0.0] * 21
        self.hand.left_y = [0.0] * 21
        self.hand.left_z = [0.0] * 21
        step = 6 * 4 # 6 fields * 4 bytes (float32)
        for i in range(0, len(sub.data), step):
            point_data = sub.data[i:i + step]
            right_x, right_y, right_z, left_x, left_y, left_z = struct.unpack('ffffff', point_data)
            self.hand.right_x[int(i/24)] = right_x
            self.hand.right_y[int(i/24)] = right_y
            self.hand.right_z[int(i/24)] = right_z
            self.hand.left_x[int(i/24)] = left_x
            self.hand.left_y[int(i/24)] = left_y
            self.hand.left_z[int(i/24)] = left_z

    def human_to_info(self):
        string = '\nHuman center:[' + str(self.human[0]) + ', ' + str(self.human[1]) + ', ' + str(self.human[2]) + ']'
        return string

    def human_sub_cb(self, sub):
        self.human = sub.data
    
    def tool_detect_completed_cb(self, msg):
        self.tool_detect_completed = msg.data

    def human_hand_completed_cb(self, msg):
        self.human_hand_completed = msg.data
        rospy.loginfo("Human and Hand detection status: " + str(self.human_hand_completed))

    def graspable_tool_cb(self, msg):
        self.find_result = msg

    def tool_axis_cb(self, msg):
        self.tool_axis = msg.data

    def move_group_feedback_cb(self, msg):
        if msg.status.text == "Solution was found and executed.":
            self.move_group_feedback = "Success"
        elif msg.status.text != "This goal has been accepted by the simple action server" and msg.status.text != None:
            self.move_group_feedback = msg.status.text

    def joint_states_cb(self, msg):
        if msg.name[0] == "finger_joint":
            self.finger_joint = msg.position[0]
    # endregion
    
    # region Tool target
    def setup_scene(self, duration): # Getting objects from simple_grasping
        grasps = self.grasps
        # Reset variables
        self.tool_detect_completed = False
        self.find_result = None
        self.table_empty = True

        msg = String()
        msg.data = self.target_tool
        self.target_tool_pub.publish(msg)
        
        start_time = time.time()
        while not self.tool_detect_completed:
            if time.time() - start_time > duration:
                print(Fore.CYAN + "Tool detect overtime. Nothing to pick up." + Style.RESET_ALL)
                break
            rospy.sleep(0.1)
        
        if self.find_result != None:
            self.table_empty = False
            print("Got result")
            print("Takes "+ Fore.CYAN + f"{time.time() - start_time:.2f}" + Style.RESET_ALL + " seconds for detection")

            # Setting scene for table
            for table in self.find_result.support_surfaces:
                table.primitives[0].dimensions = [table.primitives[0].dimensions[0], 2, 0.05]

                # Consider the robot base position, give some offset to table position
                table.primitive_poses[0].position.x += 0.1
                table.primitive_poses[0].position.y -= 0.65
                table.primitive_poses[0].position.z = 0.017 # Setting table height to 0.017
            
            length = float('-inf')
            self.object = TargetObject()
            i = 0

            for obj in self.find_result.objects:
                """
                Dimension: physical length, width, height
                Pose - Position: location in the robot's workspace
                     - Orientation: quaternion form
                """
                # print(obj.object.primitives[0].dimensions)
                if obj.object.primitives[0].dimensions[i] > length:
                    self.object.pose = obj.object.primitive_poses[0]
                    self.object.dimension = list(obj.object.primitives[0].dimensions)
                    length = self.object.dimension[i]
            
            print(Fore.CYAN + f"{self.target_tool}:\n{self.object.dimension}\n{self.object.pose}" + Style.RESET_ALL)
            object_q = [self.object.pose.orientation.x, self.object.pose.orientation.y, self.object.pose.orientation.z, self.object.pose.orientation.w]
            # print(object_q)
            object_e = trans.euler_from_quaternion(object_q)
            # print(f"degree: [{math.degrees(object_e[0])}, {math.degrees(object_e[1])}, {math.degrees(object_e[2])}]")
            
            rotation_e = []
            if self.tool_axis == "x":
                # Shift x position to tools' center
                if "Wrench" in self.target_tool:
                    print("Wrench")
                    self.object.pose.position.x += 0.04
                elif "Screwdriver" in self.target_tool:
                    self.object.pose.position.x += 0.03
                elif "Pliers" == self.target_tool:
                    self.object.pose.position.x += 0.02
                elif "Fasteners" == self.target_tool:
                    self.object.pose.position.x += 0.02
                    self.object.pose.position.z += 0.02

                while abs(math.degrees(object_e[2])) > 45:
                    print("adjust z angle")
                    object_q = trans.quaternion_multiply(object_q, trans.quaternion_from_euler(0, 0, math.pi/2))
                    self.object.pose.orientation.x, self.object.pose.orientation.y, self.object.pose.orientation.z, self.object.pose.orientation.w = object_q
                    object_e = trans.euler_from_quaternion(object_q)
                    # print(f"degree: [{math.degrees(object_e[0])}, {math.degrees(object_e[1])}, {math.degrees(object_e[2])}]")
                rotation_e = [0, math.pi, math.pi/2]
                gripper_translation = [0.015, 0.0, 0.16]
                print("Object along x axis")                
            else:
                # Shift x position to tools' center
                if "hammer" not in self.target_tool.lower():
                    self.object.pose.position.y -= 0.06
                rotation_e = [0, math.pi, math.pi/2]
                gripper_translation = [0.015, -0.01, 0.16]
                print("Object along y axis")
            
            object_q_new = trans.quaternion_multiply(object_q, trans.quaternion_from_euler(rotation_e[0], rotation_e[1], rotation_e[2]))
            
            grasps.grasp_pose.pose.position.x = self.object.pose.position.x + gripper_translation[0]
            grasps.grasp_pose.pose.position.y = self.object.pose.position.y + gripper_translation[1]
            grasps.grasp_pose.pose.position.z = self.object.pose.position.z + self.object.dimension[2] * 0.55 + gripper_translation[2]
            grasps.grasp_pose.pose.orientation.x = object_q_new[0]
            grasps.grasp_pose.pose.orientation.y = object_q_new[1]
            grasps.grasp_pose.pose.orientation.z = object_q_new[2]
            grasps.grasp_pose.pose.orientation.w = object_q_new[3]
            grasps.pre_grasp_approach.desired_distance = 0.0325
            if self.target_tool == "Fasteners":
                grasps.pre_grasp_approach.desired_distance = 0.0725
        else:
            self.table_empty = True

        msg.data = ""
        self.target_tool_pub.publish(msg)

    def go_to_pose_goal(self) -> str: # To the plan pose(on top of the target)
        move_group = self.move_group
        grasps = self.grasps
        move_group.set_max_velocity_scaling_factor(0.2)
        move_group.set_max_acceleration_scaling_factor(0.2)
        move_group.set_num_planning_attempts(10)
        move_group.set_goal_tolerance(0.01)

        pose_goal = geometry_msgs.msg.Pose()
        # pose of the tape
        pose_goal.position.x = grasps.grasp_pose.pose.position.x
        pose_goal.position.y = grasps.grasp_pose.pose.position.y
        pose_goal.position.z =  grasps.grasp_pose.pose.position.z
        pose_goal.orientation.x = grasps.grasp_pose.pose.orientation.x
        pose_goal.orientation.y = grasps.grasp_pose.pose.orientation.y
        pose_goal.orientation.z = grasps.grasp_pose.pose.orientation.z
        pose_goal.orientation.w = grasps.grasp_pose.pose.orientation.w

        move_group.set_pose_target(pose_goal)

        move_group.go(wait=True)
        move_group.stop() # For residual movement
        move_group.clear_pose_targets() # Release targets
        return self.move_group_feedback

    def approach(self, scale=1): # Planning for approaching target
        move_group = self.move_group
        grasps = self.grasps
        
        waypoints = []
        
        wpose = move_group.get_current_pose().pose
        if grasps.pre_grasp_approach.direction.vector[0] == 1:
            wpose.position.x -= scale * grasps.pre_grasp_approach.desired_distance # move in x direction
        elif grasps.pre_grasp_approach.direction.vector[1] == 1:
            wpose.position.y -= scale * grasps.pre_grasp_approach.desired_distance # move in y direction
        elif grasps.pre_grasp_approach.direction.vector[2] == 1:
            wpose.position.z -= scale * grasps.pre_grasp_approach.desired_distance # move in z direction
        elif grasps.pre_grasp_approach.direction.vector[0] == -1:
            wpose.position.x += scale * grasps.pre_grasp_approach.desired_distance # move in x direction
        elif grasps.pre_grasp_approach.direction.vector[1] == -1:
            wpose.position.y += scale * grasps.pre_grasp_approach.desired_distance # move in y direction
        elif grasps.pre_grasp_approach.direction.vector[2] == -1:
            wpose.position.z += scale * grasps.pre_grasp_approach.desired_distance # move in z direction
        else:
            wpose.position.z -= scale * grasps.pre_grasp_approach.desired_distance # move in z direction
        
        if wpose.position.z < 0.252:
            wpose.position.z = 0.25
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.001, 0.0  # waypoints to follow, eef_step, jump_threshold
        )
        return plan, fraction
    # endregion

    # region Human and hand
    def detect_target(self, duration, target):
        left = None
        right = None
        body = None
        self.hand.left_z = [0.0] * 21
        self.hand.right_z = [0.0] * 21
        self.human = None
        human_depth = float('inf')
        left_d = float('inf')
        right_d = float('inf')

        msg = Float32()
        msg.data = duration
        self.human_hand_detect_pub.publish(msg)

        start_time = time.time()
        while time.time() - start_time < duration:            
            if len(self.hand.left_z) == 21:
                for i in range(4, 24, 4):
                    if self.hand.left_z[i] != 0.0:
                        d = math.pow(self.hand.left_x[i], 2) + math.pow(self.hand.left_y[i], 2) + math.pow(self.hand.left_z[i], 2)
                        if d < left_d:
                            left_d = d
                            left = ["Left", self.hand.left_x[i], self.hand.left_y[i], self.hand.left_z[i]]
            if len(self.hand.right_z) == 21:
                for i in range(4, 24, 4):
                    if self.hand.right_z[i] != 0.0:
                        d = math.pow(self.hand.right_x[i], 2) + math.pow(self.hand.right_y[i], 2) + math.pow(self.hand.right_z[i], 2)
                        if d < right_d:
                            right_d = d
                            right = ["Right", self.hand.right_x[i], self.hand.right_y[i], self.hand.right_z[i]]
            if self.human != None:
                if self.human[2] != 0.0 and self.human[2] < human_depth:
                    body = ["Human", self.human[0], self.human[1], self.human[2]]
                    human_depth = self.human[2]
        
        msg.data = 0.0
        self.human_hand_detect_pub.publish(msg)

        final_target = []
        if target == 'Left':
            final_target = [left, right, body]
        elif target == "Right":
            final_target = [right, left, body]
        else:
            final_target = [body, right, left]
        
        print(final_target)       

        for i in final_target:
            if i != None:
                if i[3] != 0.0:
                    if i[0] == "Left":
                        print(Fore.GREEN + "Left Hand:")                
                    elif i[0] == "Right":
                        print(Fore.BLUE + "Right Hand:")
                    else:
                        print(Fore.YELLOW + "Human center:")
                        i[2] = i[2] * 0.7
                    print(i[1:])
                    return i[1:]
        return None
    
    def cal_human_q(self, new_human_position):
        # 20 degree angle with x distance
        gripper_translation = [-0.25, 0.0, 0.3] #-0.35, -0.2, 0.2
        grasps = self.grasps
        # To desired human position
        grasps.grasp_pose.pose.position.x = new_human_position[0]*math.cos(math.radians(20)) + gripper_translation[0]
        grasps.grasp_pose.pose.position.y = new_human_position[1] + gripper_translation[1]
        grasps.grasp_pose.pose.position.z = new_human_position[2] + gripper_translation[2]
        
        grasps.grasp_pose.pose.orientation.x = -0.7130000824323441
        grasps.grasp_pose.pose.orientation.y = 0.7011493154633497
        grasps.grasp_pose.pose.orientation.z = -0.0013575302716290754
        grasps.grasp_pose.pose.orientation.w = 0.0043216881317590565
        
        # rotate pose
        # grasps.grasp_pose.pose.orientation.x = -0.473061463918355
        # grasps.grasp_pose.pose.orientation.y = 0.5296206918637303
        # grasps.grasp_pose.pose.orientation.z = -0.5094212215352387
        # grasps.grasp_pose.pose.orientation.w = 0.4860090463713115

        print("======cal_human_q======\n", grasps.grasp_pose.pose)

    def approach_human(self, scale=1):
        move_group = self.move_group
        grasps = self.grasps
        move_group.set_planning_time(3.0)
        move_group.set_max_velocity_scaling_factor(0.2)
        move_group.set_max_acceleration_scaling_factor(0.1)
        joint_goal = move_group.get_current_joint_values()

        waypoints = [grasps.grasp_pose.pose]

        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow, eef_step, jump_threshold
        )

        return plan, fraction
    # endregion
    
    # region Package movement of interacting with master main script    
    def pickup_cb(self, msg):
        feedback = String()
        s = msg.data.split("_")
        if len(s) > 1: # Scan_target tool_duration
            self.target_tool = tool_replace(s[1])
            print("\nTarget:", self.target_tool)
            self.pre_grasp_posture()

            scan = ['center', 'left', 'right']
            for i in scan:
                result = self.scaning_pose(i)
                if result != "Success":
                    feedback.data = "Scan " + result
                    self.pickup_feedback_pub.publish(feedback)
                    return
                rospy.sleep(1)
                self.setup_scene(int(s[2]))
                if not self.table_empty:
                    break
            
            feedback.data = "Object " + self.move_group_feedback
            self.pickup_feedback_pub.publish(feedback)
        elif s[0] == "Object":
            if not self.table_empty:
                result = self.go_to_pose_goal()
                if result != "Success":
                    feedback.data = "Object " + result
                    self.pickup_feedback_pub.publish(feedback)
                    return
                feedback.data = "Approach " + self.move_group_feedback
                self.pickup_feedback_pub.publish(feedback)
            else:
                feedback.data = "No_Target"
                self.starting_pose()
                self.pickup_feedback_pub.publish(feedback)
        elif s[0] == "Approach":
            if not self.table_empty:
                approach, fraction = self.approach(scale=0.42)
                result = self.execute_plan(approach)
                if result != "Success":
                    feedback.data = "Approach " + result
                    self.pickup_feedback_pub.publish(feedback)
                    return
                rospy.sleep(1)
                self.grasp_posture()
                rospy.sleep(1)
                print("Gripper: ", self.finger_joint)
                if self.finger_joint >= 0.78: # Fully close
                    feedback.data = "Scan Failed"
                    self.pickup_feedback_pub.publish(feedback)
                    return
                feedback.data = "Inhand " + self.move_group_feedback
                self.pickup_feedback_pub.publish(feedback)
        elif s[0] == "Inhand":
            if not self.table_empty:
                if self.target_tool == "Fasteners":
                    result = self.starting_pose()
                else:
                    result = self.tool_inhand_pose()
                if result != "Success":
                    feedback.data = "Inhand " + result
                    self.pickup_feedback_pub.publish(feedback)
                    return                
                feedback.data = self.move_group_feedback
                self.pickup_feedback_pub.publish(feedback)
                print("++++++pick up end++++++")

    def handoff_cb(self, msg):
        feedback = String()
        s = msg.data.split("_")
        if len(s) > 1: # detect target_duration
            human = self.detect_target(int(s[2]), s[1])
            if human != None:
                human_new = human_cam_trans(human)
                self.cal_human_q(human_new)
                print(Style.RESET_ALL)

                approach_human, fraction = self.approach_human()
                result = self.execute_plan(approach_human)
                if result != "Success":
                    feedback.data = "Go " + result
                    self.handoff_feedback_pub.publish(feedback)
                    return
                rospy.sleep(1)
                input('open:')
                self.pre_grasp_posture()
                feedback.data = "Start " + result
                self.handoff_feedback_pub.publish(feedback)
            else:
                feedback.data = "NoTarget"
                self.handoff_feedback_pub.publish(feedback)
        else:
            result = self.starting_pose()
            if result != "Success":
                feedback.data = "Start " + result
                self.handoff_feedback_pub.publish(feedback)
                return
            feedback.data = result
            self.handoff_feedback_pub.publish(feedback)
            print("======hand off end======")
    
    def reset_cb(self, msg):
        print(f"Receive {msg.data}, processing...")
        if msg.data == "Open":
            self.pre_grasp_posture()
        elif msg.data == "Close":
            self.grasp_posture()
        elif "Scan" in msg.data:
            str = msg.data.split('_')
            self.scaning_pose(str[1])
        elif msg.data =="Start":
            self.starting_pose()
        elif msg.data == "Tool_Stop":
            stop_msg = String()
            stop_msg.data = ""
            self.target_tool_pub.publish(stop_msg)
        elif "Tool" in msg.data:
            str = msg.data.split('_')
            self.target_tool = str[1]
            self.setup_scene()    
        elif "Print" == msg.data:
            move_group = self.move_group
            print("\nCurrent Pose:\n", move_group.get_current_joint_values())
            print(move_group.get_current_pose().pose)
        elif "Video" in msg.data:
            str = msg.data.split('_')
            self.video(str[1])

        self.reset_feedback_pub.publish("Completed")   
        print("Completed!")
    # endregion

if __name__ == "__main__":
    try:
        arm()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Interrupted")
        