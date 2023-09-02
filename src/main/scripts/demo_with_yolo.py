#!/usr/bin/env python3

# UR5 arms path planning for human gesture detecting with YOLO
# Author : Charlotte Wang

import os
import sys
import time
import copy
import rospy
import random
import numpy as np
import moveit_msgs.msg
import moveit_commander
import geometry_msgs.msg

from moveit_msgs.msg import Grasp
from grasping_msgs.msg import FindGraspableObjectsResult
from std_msgs.msg import Float32MultiArray, String, Bool, Float32
from sensor_msgs.msg import PointCloud2
from colorama import Fore, Back, Style
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import Constraints, JointConstraint, MoveGroupActionFeedback
from tf import transformations as trans
import math
import struct

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
    # 18 degree tiled with 0.85 radius of the human camera = 0.276m
    offset = np.array([-0.2, -0.2, 0.265])
    output = np.add(offset, input)   

    print("New coordinate respect to base:\n", output)
    return output

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

class movetodesiredposition(object) :
    def __init__(self):
        rospy.init_node("arm_node")
        super(movetodesiredposition, self).__init__()
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
        self.move_group_feedback = None
        
        # Display trajectories in Rviz:
        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        self.move_group_feedback_sub = rospy.Subscriber(
            '/move_group/feedback',
            MoveGroupActionFeedback,
            self.move_group_feedback_callback
        )
        

        # For sending and receive signal from tool detection
        self.target_tool_pub = rospy.Publisher(
            '~target_tool',
            String,
            queue_size=1
        )        
        self.graspable_result_sub = rospy.Subscriber(
            '/tool_detect_node/graspable_tool',
            FindGraspableObjectsResult,
            self.graspable_tool_callback
        )
        self.tool_detect_completed_sub = rospy.Subscriber(
            '/tool_detect_node/completed',
            Bool, 
            self.tool_detect_completed_callback
        )
        self.tool_detect_completed_sub = rospy.Subscriber(
            '/tool_detect_node/tool_axis',
            String, 
            self.tool_axis_callback
        )     

        # For sending and receive signal and data from human and hand detection
        self.human_hand_detect_pub = rospy.Publisher(
            '~human_hand_detect',
            Float32,
            queue_size=1
        )
        self.human_sub = rospy.Subscriber(
            '/human_hand_node/human_info', 
            Float32MultiArray, 
            self.human_sub_callback
        )
        self.hand_sub = rospy.Subscriber(
            '/human_hand_node/hand_info', 
            PointCloud2, 
            self.hand_sub_callback
        )
        self.human_hand_completed_sub = rospy.Subscriber(
            '/human_hand_node/completed',
            Bool, 
            self.human_hand_completed_callback
        )
        self.human_hand_completed = False

        planning_frame = move_group.get_planning_frame()
        eef_link = move_group.get_end_effector_link()
        group_names = robot.get_group_names()

        # region System Info
        # print("======System Info======")        
        # print("Planning frame: %s" % planning_frame)
        # print("End effector link: %s" % eef_link)
        # print("Available Planning Groups:", group_names)
        # print("======End System Info======")
        # print("======Robot State======")
        print("\nCurrent Pose:\n", move_group.get_current_joint_values())
        print(move_group.get_current_pose().pose)
        # print("======End Robot State======")

        input("------Press Enter to proceed------")
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
        # endregion

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

        # region Constrains
        constraints = Constraints()
        elbow_joint_constrain = JointConstraint()
        elbow_joint_constrain.joint_name = 'elbow_joint'
        elbow_joint_constrain.position = 0.0
        elbow_joint_constrain.tolerance_below = 15 # unit: degree
        elbow_joint_constrain.tolerance_above = 135
        elbow_joint_constrain.weight = 0.7
        constraints.joint_constraints.append(elbow_joint_constrain)
        
        shoulder_lift_joint_constrain = JointConstraint()
        shoulder_lift_joint_constrain.joint_name = 'shoulder_lift_joint'
        shoulder_lift_joint_constrain.position = 0.0
        shoulder_lift_joint_constrain.tolerance_above = -10
        constraints.joint_constraints.append(shoulder_lift_joint_constrain)

        # shoulder_pan_joint = JointConstraint()
        # shoulder_pan_joint.joint_name = 'shoulder_pan_joint'
        # shoulder_pan_joint.position = 0.0
        # shoulder_pan_joint.tolerance_above = 50
        # constraints.joint_constraints.append(shoulder_pan_joint)

        move_group.set_path_constraints(constraints)
        # endregion

        self.grasps = grasps
        self.scene.remove_world_object()
        self.table_empty = False

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
    
    def starting_pose(self): # Go to the starting pose
        move_group = self.move_group
        move_group.set_max_velocity_scaling_factor(0.3)
        move_group.set_max_acceleration_scaling_factor(0.3)
        print("\nStarting Pose:", move_group.get_current_joint_values())
        print(move_group.get_current_pose().pose)

        # Starting pose
        joint_goal = [0.008436894975602627, -2.2472408453570765, 1.9655742645263672, -1.2976849714862269, -1.5677931944476526, -0.011224095021383107]
        move_group.go(joint_goal, wait=True)
        rospy.sleep(0.1)
        print("feedback:", self.move_group_feedback)
        for i in range(3):
            if self.move_group_feedback == "NO_IK_SOLUTION":
                print("Received NO_IK_SOLUTION, execute starting_pose again")
                move_group.go(joint_goal, wait=True)
            else: break
        move_group.stop()
    
    def scaning_pose(self): # Go to the starting pose
        move_group = self.move_group
        move_group.set_max_velocity_scaling_factor(0.3)
        move_group.set_max_acceleration_scaling_factor(0.3)

        # Starting pose
        joint_goal = [-0.1874778906451624, -1.6312454382525843, 1.2965798377990723, -1.2432053724872034, -1.5650742689715784, -0.20414525667299444]
        move_group.go(joint_goal, wait=True)
        print(self.move_group_feedback)
        rospy.sleep(0.1)
        for i in range(3):
            if self.move_group_feedback == "NO_IK_SOLUTION":
                print("Received NO_IK_SOLUTION, execute starting_pose again")
                move_group.go(joint_goal, wait=True)
            else: break
        move_group.stop()

    def setup_scene(self): # Getting objects from simple_grasping
        grasps = self.grasps
        msg = String()
        msg.data = self.target_tool # Screwdriver, Hammer, Pliers
        self.target_tool_pub.publish(msg)
        
        start_time = time.time()
        while not self.tool_detect_completed:
            rospy.sleep(0.1)
        
        if self.find_result != None:
            print("Got result")
            print("Takes "+ Fore.CYAN + f"{time.time() - start_time:.2f}" + Style.RESET_ALL + " seconds for detection")

            # Setting scene for table
            for table in self.find_result.support_surfaces:
                # dimension: [Length, Width, Height] of the viewsight, setting the dimention wider for pick and place
                # print(f"Table:{table.primitives[0].dimensions}\n{table.primitive_poses[0]}")
                table.primitives[0].dimensions = [table.primitives[0].dimensions[0], 2, 0.05]

                # Consider the robot base position, give some offset to table position
                table.primitive_poses[0].position.x += 0.1
                table.primitive_poses[0].position.y -= 0.65
                table.primitive_poses[0].position.z = 0.017 # Setting table height to 0.017
                
            # Setting object's pose and size
            msg.data = ""
            self.target_tool_pub.publish(msg)

            length = float('-inf')
            self.object = TargetObject()
            i = 0
            if self.target_tool == "bolts":
                i = 2

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
            print(object_q)
            object_e = trans.euler_from_quaternion(object_q)
            print(f"degree: [{math.degrees(object_e[0])}, {math.degrees(object_e[1])}, {math.degrees(object_e[2])}]")
            
            rotation_e = []
            if self.tool_axis == "x":
                # Shift x position to tools' center
                if "hammer" not in self.target_tool.lower():
                    self.object.pose.position.x += 0.06
                elif "hammer" in self.target_tool.lower():
                    self.object.pose.position.x += 0.02
                while abs(math.degrees(object_e[2])) > 45:
                    print("adjust z angle")
                    object_q = trans.quaternion_multiply(object_q, trans.quaternion_from_euler(0, 0, math.pi/2))
                    self.object.pose.orientation.x, self.object.pose.orientation.y, self.object.pose.orientation.z, self.object.pose.orientation.w = object_q
                    object_e = trans.euler_from_quaternion(object_q)
                    print(f"degree: [{math.degrees(object_e[0])}, {math.degrees(object_e[1])}, {math.degrees(object_e[2])}]")
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
            object_e_new = trans.euler_from_quaternion(object_q_new)
            
            grasps.grasp_pose.pose.position.x = self.object.pose.position.x + gripper_translation[0]
            grasps.grasp_pose.pose.position.y = self.object.pose.position.y + gripper_translation[1]
            grasps.grasp_pose.pose.position.z = self.object.pose.position.z + self.object.dimension[2] * 0.55 + gripper_translation[2]
            grasps.grasp_pose.pose.orientation.x = object_q_new[0]
            grasps.grasp_pose.pose.orientation.y = object_q_new[1]
            grasps.grasp_pose.pose.orientation.z = object_q_new[2]
            grasps.grasp_pose.pose.orientation.w = object_q_new[3]
            grasps.pre_grasp_approach.desired_distance = 0.0325
            # print("======New grasp pose======\n", grasps.grasp_pose.pose)
            # print(f"new degree: [{math.degrees(object_e_new[0])}, {math.degrees(object_e_new[1])}, {math.degrees(object_e_new[2])}]")
        else:
            print("Nothing to pick up")
            msg.data = ""
            self.target_tool_pub.publish(msg)
            self.table_empty = True          

    # region Approaching target
    def go_to_pose_goal(self): # To the plan pose(on top of the target)
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
        for i in range(3):
            if self.move_group_feedback == "NO_IK_SOLUTION":
                print("Received NO_IK_SOLUTION, execute go_to_pose_goal again")
                move_group.go(wait=True)
            else: break
        move_group.stop() # For residual movement
        move_group.clear_pose_targets() # Release targets

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
        # otherwise default to +z direction
        else:
            wpose.position.z -= scale * grasps.pre_grasp_approach.desired_distance # move in z direction
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.001, 0.0  # waypoints to follow, eef_step, jump_threshold
        )
        return plan, fraction
    # endregion

    # region Approach human
    def cal_human_q(self, new_human_position):
        gripper_translation = [-0.2, 0.0, 0.0] # Consider gripper and pickup object's height
        grasps = self.grasps
        # To desired human position
        grasps.grasp_pose.pose.position.x = new_human_position[0] + gripper_translation[0]
        grasps.grasp_pose.pose.position.y = new_human_position[1] + gripper_translation[1]
        grasps.grasp_pose.pose.position.z = new_human_position[2] + gripper_translation[2]
        
        if self.target_tool == "bolts":
            grasps.grasp_pose.pose.orientation.x = -0.7130000824323441
            grasps.grasp_pose.pose.orientation.y = 0.7011493154633497
            grasps.grasp_pose.pose.orientation.z = -0.0013575302716290754
            grasps.grasp_pose.pose.orientation.w = 0.0043216881317590565
        else:
            grasps.grasp_pose.pose.orientation.x = -0.473061463918355
            grasps.grasp_pose.pose.orientation.y = 0.5296206918637303
            grasps.grasp_pose.pose.orientation.z = -0.5094212215352387
            grasps.grasp_pose.pose.orientation.w = 0.4860090463713115

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

    def detect_target(self, duration, target):
        right = None
        left = None
        body = None
        human_depth = float('inf')

        msg = Float32()
        msg.data = duration
        self.human_hand_detect_pub.publish(msg)

        start_time = time.time()
        while time.time() - start_time < duration:
            for i in range(4, 24, 4):
                if self.hand.right_z[i] != 0.0:
                    right = ["Right", self.hand.right_x[i], self.hand.right_y[i], self.hand.right_z[i]]
                if self.hand.left_z[i] != 0.0:
                    left = ["Left", self.hand.left_x[i], self.hand.left_y[i], self.hand.left_z[i]]
            if self.human[2] != 0.0 and self.human[2] < human_depth:
                body = ["Human", self.human[0], self.human[1], self.human[2]]
                human_depth = self.human[2]
        
        msg.data = False
        self.human_hand_detect_pub.publish(msg)

        final_target = []
        if target == "Right":
            final_target = [right, left, body]
        elif target == 'Left':
            final_target = [left, right, body]
        else:
            final_target = [body, right, left]
        
        print(final_target)       

        for i in final_target:
            if i != None:
                if i[3] != 0.0:                
                    if i[0] == "Right":
                        print(Fore.BLUE + "Right Hand:")
                    elif i[0] == "Left":
                        print(Fore.GREEN + "Left Hand:")
                    else:
                        print(Fore.YELLOW + "Human center:")
                        i[2] = i[2] * 0.7
                    print(i[1:])
                    return i[1:]
        return None
    # endregion

    # region Display and Execute plan
    def display_trajectory(self, plan):
        robot = self.robot

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        rate = rospy.Rate(1)
        
        self.display_trajectory_publisher.publish(display_trajectory)
        rate.sleep()

    def execute_plan(self, plan):
        move_group = self.move_group

        move_group.execute(plan, wait=True)

        return -1
    # endregion

    # region Subscriber
    def hand_to_info(self, num):
        string = '\nRight hand:[' + str(self.hand.right_x[num]) + ', ' + str(self.hand.right_y[num]) + ', ' + str(self.hand.right_z[num]) + ']'
        string += '\nLeft hand:[' + str(self.hand.left_x[num]) + ', ' + str(self.hand.left_y[num]) + ', ' + str(self.hand.left_z[num]) + ']'
        return string

    def hand_sub_callback(self, sub):
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

    def human_sub_callback(self, sub):
        self.human = sub.data
    
    def tool_detect_completed_callback(self, msg):
        self.tool_detect_completed = msg.data
        rospy.loginfo("Tool detection status: " + str(self.tool_detect_completed))

    def human_hand_completed_callback(self, msg):
        self.human_hand_completed = msg.data
        rospy.loginfo("Human and Hand detection status: " + str(self.human_hand_completed))

    def graspable_tool_callback(self, msg):
        self.find_result = msg

    def tool_axis_callback(self, msg):
        self.tool_axis = msg.data

    def move_group_feedback_callback(self, msg):
        self.move_group_feedback = msg.status.text

    # endregion

def pick():
    try:
        while not rospy.is_shutdown():
            plantherobot = movetodesiredposition()

            # Go to initial pose and detect object
            plantherobot.pre_grasp_posture()
            plantherobot.starting_pose()
            
            tool_list = ["Screwdriver", "Hammer", "Pliers"]
            plantherobot.target_tool = input("++++++Input target tool:++++++\n")
            # random.choice(tool_list)
            # input("++++++Input target tool:++++++\n") # Screwdriver, Hammer, Pliers
            plantherobot.scaning_pose()
            plantherobot.setup_scene()

            input("go to goal")

            if not plantherobot.table_empty:
                # input("------Press Enter to go pose goal------")
                # Goes to target's top
                plantherobot.go_to_pose_goal()
                
                # input("Grasp")
                # Goes down to the target
                approach, fraction = plantherobot.approach(scale=0.425)
                plantherobot.display_trajectory(approach)
                plantherobot.execute_plan(approach)
                rospy.sleep(1)
                plantherobot.grasp_posture()
            
            rospy.sleep(1)
            # input('Back to starting pose')
            plantherobot.starting_pose()            

            target = "Right" # Main target
            duration = 3 # Seconds for detection

            input(f"------Press Enter to detect {target} for {duration} seconds------")
            human = plantherobot.detect_target(duration, target)
            # human = [-0.1409953385591507, 0.10165759176015854, 0.893000066280365]
            if human != None:            
                human_new = human_cam_trans(human)
                plantherobot.cal_human_q(human_new)
                print(Style.RESET_ALL)

                approach_human, fraction = plantherobot.approach_human()
                input("Watch the plan path")
                plantherobot.display_trajectory(approach_human)
                input("Go plan path")
                plantherobot.execute_plan(approach_human)
            break

    except KeyboardInterrupt:
        rospy.loginfo("Interrupted")

if __name__ == "__main__":
    pid = os.getpid()
    with open('pid.sh', 'w') as launch_file:
        launch_file.write(f'kill -9 {pid}')

    pick()
        
