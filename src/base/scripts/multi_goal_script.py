#!/usr/bin/env python3

import rospy
import math

import actionlib
from nav_msgs.srv import GetPlan
from std_msgs.msg import String, Bool
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
from angles import shortest_angular_distance


class MoveBaseSeq():

    def __init__(self):       
        
        self.goal_published = False
        
        # Create a publisher for the /goal_achieved topic
        
        self.xy_goal_tolerance  = rospy.get_param('/xy_goal_tolerance', 0.1)
        self.yaw_goal_tolerance = rospy.get_param('/yaw_goal_tolerance', 0.05)
        
        self.goal_achieved_pub = rospy.Publisher('/goal_achieved', String, queue_size=10)

        
        self.tool_flag=False
        
        self.tool_goal_msg=False

        self.goal_reached = False
        
        # init setPrevPose to 0
        self.setPrevPose = 0
        
        self.goal = MoveBaseGoal()
        # self.movebase_client()

        """
        Get the value from the Parameter Server
        """
        points_left_seq = rospy.get_param('multi_goal_navigation/p_left_seq')
        points_right_seq = rospy.get_param('multi_goal_navigation/p_right_seq')
        points_start_seq = rospy.get_param('multi_goal_navigation/p_start_seq')
        
        # Only yaw angle required (no ratotions around x and y axes) in deg:
        
        yaweulerangles_left_seq = rospy.get_param('multi_goal_navigation/yea_left_seq')
        yaweulerangles_right_seq = rospy.get_param('multi_goal_navigation/yea_right_seq')
        yaweulerangles_start_seq = rospy.get_param('multi_goal_navigation/yea_start_seq')
        
        """
        Preprocess the list recieved
        """
        
        #List of goal quaternions:
        quat_left_seq = list()
        quat_right_seq = list()
        quat_start_seq = list()
        #List of goal poses:
        self.pose_seq=Pose()
        self.pose_left_seq = list()
        self.pose_right_seq = list()
        self.pose_start_seq = list()
        self.goal_cnt = 0
        
        for yawangle in yaweulerangles_left_seq:
            #Unpacking the quaternion tuple and passing it as arguments to Quaternion message constructor
            quat_left_seq.append(Quaternion(*(quaternion_from_euler(0, 0, yawangle*math.pi/180, axes='sxyz'))))
        for yawangle in yaweulerangles_right_seq:
            #Unpacking the quaternion tuple and passing it as arguments to Quaternion message constructor
            quat_right_seq.append(Quaternion(*(quaternion_from_euler(0, 0, yawangle*math.pi/180, axes='sxyz'))))
        for yawangle in yaweulerangles_start_seq:
            #Unpacking the quaternion tuple and passing it as arguments to Quaternion message constructor
            quat_start_seq.append(Quaternion(*(quaternion_from_euler(0, 0, yawangle*math.pi/180, axes='sxyz'))))
        n = 3
        # Returns a list of lists [[point1], [point2],...[pointn]]
        points_left = [points_left_seq[i:i+n] for i in range(0, len(points_left_seq), n)]
        points_right = [points_right_seq[i:i+n] for i in range(0, len(points_right_seq), n)]
        points_start = [points_start_seq[i:i+n] for i in range(0, len(points_start_seq), n)]
        # rospy.loginfo(str(points))
        for point in points_left:
            #Exploit n variable to cycle in quat_seq
            pose_stamped = PoseStamped()
            pose_stamped.pose.position = Point(*point)  # Populate position using the 'point' variable
            pose_stamped.pose.orientation = quat_left_seq[n - 3]  # Assuming 'quat_start_seq' contains quaternion data
            pose_stamped.header.stamp = rospy.Time.now()  # Current time
            pose_stamped.header.frame_id = "t265_odom_frame" 
            self.pose_left_seq.append(Pose(Point(*point),quat_left_seq[n-3]))
            # self.pose_left_seq.append(pose_stamped)
            n += 1
        n = 3
        for point in points_right:
            #Exploit n variable to cycle in quat_seq
            pose_stamped = PoseStamped()
            pose_stamped.pose.position = Point(*point)  # Populate position using the 'point' variable
            pose_stamped.pose.orientation = quat_right_seq[n - 3]  # Assuming 'quat_start_seq' contains quaternion data
            pose_stamped.header.stamp = rospy.Time.now()  # Current time
            pose_stamped.header.frame_id = "t265_odom_frame" 
            self.pose_right_seq.append(Pose(Point(*point),quat_right_seq[n-3]))
            # self.pose_right_seq.append(pose_stamped)
            n += 1
            
        n = 3
        for point in points_start:
            #Exploit n variable to cycle in quat_seq
            pose_stamped = PoseStamped()
            pose_stamped.pose.position = Point(*point)  # Populate position using the 'point' variable
            # print(quat_start_seq[n - 3])
            pose_stamped.pose.orientation = quat_start_seq[n - 3]  # Assuming 'quat_start_seq' contains quaternion data
            pose_stamped.header.stamp = rospy.Time.now()  # Current time
            pose_stamped.header.frame_id = "t265_odom_frame"
            self.pose_start_seq.append(Pose(Point(*point),quat_start_seq[n-3]))
            # self.pose_start_seq.append(pose_stamped)
            n += 1
        
        """
        Create a client and wait for the server response
        """
        #Create action client
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        
        #wait = self.client.wait_for_server(rospy.Duration(5.0))
        
        wait = self.client.wait_for_server()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.loginfo("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")
    
        
        #rospy.loginfo(str(self.pose_seq))
        
        self.total_path_time=0.0
        time_val1=0
        time_val2=0
        time_val3=0
        count1=0
        count2=0
        # for itr in range(0,len(self.pose_right_seq)):
        #     while (itr<=2):
        #         count1+=1
        #         print(self.total_path_time,"Total path time")
        #         print(time_val1)
        #         time_val1=self.calculate_total_time(self.pose_right_seq[itr], self.pose_right_seq[itr+1])
            
        # for itr in range(0,len(self.pose_left_seq)):
        #     while (itr<=2):
        #         count2+=1
        #         print(count2)
        #         time_val2 =self.calculate_total_time(self.pose_left_seq[itr], self.pose_left_seq[itr+1])
            
        # for itr in range(0,len(self.pose_start_seq)):
        #     while (itr<=2):
        #         time_val3 =self.calculate_total_time(self.pose_start_seq[itr], self.pose_start_seq[itr+1])
                
            
        # print(self.total_path_time)

        rospy.loginfo("Starting Pickup")

        # while not rospy.is_shutdown():
        #     # Publish the goal status
        # while True:

        self.subscriber_node()
        # self.movebase_client()
        
        
    def calculate_total_time(self, start_pose, goal_pose ):
        # Call the make_plan service to get the path
        # rospy.wait_for_service('make_plan')
        try:
            get_plan = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
            response = get_plan(start_pose, goal_pose, 0.0)  # Use 0.0 as the tolerance for simplicity

            if len(response.plan.poses) > 0:
                # Assuming the robot's maximum velocity is 1.0 (you can replace this with the actual value)
                robot_max_velocity = 1.0
                estimated_time = self.estimate_time(response.plan, robot_max_velocity)
                # rospy.loginfo("Estimated time to follow the path: %.2f seconds", estimated_time)
                print(estimated_time)
                return estimated_time

            else:
                rospy.logwarn("Failed to find a valid path!")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", str(e))
        
    def calculate_distance(self, pose1, pose2):
        # Calculate Euclidean distance between two poses
        dx = pose1.pose.position.x - pose2.pose.position.x
        dy = pose1.pose.position.y - pose2.pose.position.y
        dz = pose1.pose.position.z - pose2.pose.position.z
        return (dx**2 + dy**2 + dz**2)**0.5

    def estimate_time(self, path_msg, robot_max_velocity):
        total_distance = 0.0

        # Calculate the total distance of the path
        for i in range(1, len(path_msg.poses)):
            total_distance += self.calculate_distance(path_msg.poses[i-1], path_msg.poses[i])

        # Estimate the time taken for the path
        time_estimated = total_distance / robot_max_velocity

        return time_estimated
        
    """
    This parameter is an optional callback function that gets called when the action goal becomes active 
    (i.e., when the server starts working on the goal).
    """

    def active_cb(self):
        rospy.loginfo("Goal pose "+str(self.goal_cnt+1)+" is now being processed by the Action Server...")
        
    """
    This parameter is an optional callback function that gets called when the action server sends feedback during the execution of the goal. 
    The feedback contains information about the current progress of the action.
    """

    def feedback_cb(self, feedback):
        pass
        # rospy.loginfo("Feedback for goal "+str(self.goal_cnt)+": "+str(feedback))
        # rospy.loginfo("Feedback for goal pose "+str(self.goal_cnt+1)+" received")
        
        # self.checkPose = feedback.base_position.pose

        
        
        # if self.setPrevPose == 0:
        #     rospy.loginfo('still moving 1')
        #     distance = (feedback.base_position.pose.position.x - self.goal.target_pose.pose.position.x)**2
        #     distance += (feedback.base_position.pose.position.y - self.goal.target_pose.pose.position.y)**2
        #     distance = math.sqrt(distance)

        #     if (distance > self.xy_goal_tolerance):
        #         return
            
        #     rospy.loginfo('still moving 2')

        #     goal_yaw = euler_from_quaternion([self.goal.target_pose.pose.orientation.x, \
        #                                       self.goal.target_pose.pose.orientation.y, \
        #                                       self.goal.target_pose.pose.orientation.z, \
        #                                       self.goal.target_pose.pose.orientation.w])[2]

        #     base_yaw = euler_from_quaternion([feedback.base_position.pose.orientation.x, \
        #                                       feedback.base_position.pose.orientation.y, \
        #                                       feedback.base_position.pose.orientation.z, \
        #                                       feedback.base_position.pose.orientation.w])[2]

        #     ang_distance = abs(shortest_angular_distance(goal_yaw, base_yaw))

        #     rospy.loginfo('still moving 3')

        #     if (ang_distance > self.yaw_goal_tolerance):
        #         return

        #     rospy.loginfo('still moving 4')

        #     # wait until there is no change in pose and yaw
        #     if (self.setPrevPose == 0):
        #         rospy.loginfo('send rotation')
        #         rospy.loginfo('still moving 5')
                
        #         self.setPrevPose = 1
        #         # save previous pose
        #         self.prevPose = feedback.base_position.pose
        #         return
        # else:
        #     rospy.loginfo('completely stop 1')
        #     # see if robot stopped completely
        #     distance = (feedback.base_position.pose.position.x - self.prevPose.position.x)**2
        #     distance += (feedback.base_position.pose.position.y - self.prevPose.position.y)**2
        #     distance = math.sqrt(distance)

        #     if (distance > 0.05):
        #         self.prevPose = feedback.base_position.pose
        #         return
        #     rospy.loginfo('completely stop 2')

        #     prev_yaw = euler_from_quaternion([self.prevPose.orientation.x, \
        #                                       self.prevPose.orientation.y, \
        #                                       self.prevPose.orientation.z, \
        #                                       self.prevPose.orientation.w])[2]

        #     base_yaw = euler_from_quaternion([feedback.base_position.pose.orientation.x, \
        #                                       feedback.base_position.pose.orientation.y, \
        #                                       feedback.base_position.pose.orientation.z, \
        #                                       feedback.base_position.pose.orientation.w])[2]

        #     ang_distance = abs(shortest_angular_distance(prev_yaw, base_yaw))

        #     rospy.loginfo('completely stop 3')


        #     if (ang_distance > 0.01):
        #         self.prevPose = feedback.base_position.pose
        #         return

        #     rospy.loginfo('completely stop 4')

        #     self.goal_cnt += 1
        #     if self.goal_cnt == len(self.pose_seq)-1:
        #         self.goal_cnt = len(self.pose_seq)-1

        #     self.goal.target_pose.header.frame_id = "t265_odom_frame"
        #     self.goal.target_pose.header.stamp = rospy.Time.now()
        #     self.goal.target_pose.pose = self.pose_seq[self.goal_cnt]
        #     rospy.loginfo("Sending goal pose "+str(self.goal_cnt)+" to Action Server")
        #     rospy.loginfo(str(self.pose_seq[self.goal_cnt]))

        #     # Sleep 2 secs
        #     # rospy.sleep(2.)
        #     self.client.send_goal(self.goal, feedback_cb=self.feedback_cb)

        #     self.setPrevPose = 0


        
    """
    This parameter is an optional callback function that gets called when the action goal is completed (either successfully or with an error). 
    The callback takes two arguments: gh (GoalHandle) and result (action result message type).
    """

    def done_cb(self, status, result):
        self.goal_cnt += 1
        # Reference for terminal status values: http://docs.ros.org/diamondback/api/actionlib_msgs/html/msg/GoalStatus.html
        if status == 2:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request after it started executing, completed execution!")

        if status == 3:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" reached") 
            
            if self.goal_cnt< len(self.pose_seq):
                next_goal = MoveBaseGoal()
                next_goal.target_pose.header.frame_id = "t265_odom_frame"
                next_goal.target_pose.header.stamp = rospy.Time.now()
                next_goal.target_pose.pose = self.pose_seq[self.goal_cnt]
                rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
                rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
                self.client.send_goal(next_goal, self.done_cb, self.active_cb, self.feedback_cb) 
            else:
                print(self.goal_reached)

                # Simulate reaching the goal and publishing the message

                # while not rospy.is_shutdown():
                #     # Publish the goal status
                if self.goal_reached=="A_Reached":
                    self.goal_achieved_pub.publish("A_Success")
                elif self.goal_reached=="B_Reached":
                    self.goal_achieved_pub.publish("B_Success")
                elif self.tool_flag:
                    self.goal_achieved_pub.publish("Tool_Success")
            
                rospy.loginfo("Final goal pose reached!")
                return

        if status == 4:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" was aborted by the Action Server")
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" aborted, shutting down!")
            self.goal_achieved_pub.publish("Aborted")
            return

        if status == 5:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" has been rejected by the Action Server")
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" rejected, shutting down!")
            return

        if status == 8:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request before it started executing, successfully cancelled!")
            
        
           

    def movebase_client(self,msg):
        self.goal_cnt=0
        self.goal_reached=""
        self.tool_flag=False
        msg.data=="Will Go Left"
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "t265_odom_frame"
        goal.target_pose.header.stamp = rospy.Time.now() 
        print("This is the message recieved",msg.data)
        
        #for pose in pose_seq:   
        if msg.data=="Will Go Right" and not self.goal_published:
            self.goal_reached=True
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "t265_odom_frame"
            goal.target_pose.header.stamp = rospy.Time.now() 
            goal.target_pose.pose = self.pose_left_seq[self.goal_cnt]
            rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
            rospy.loginfo(str(self.pose_left_seq[self.goal_cnt]))
            self.pose_seq=self.pose_left_seq
            self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
        elif msg.data=="Will Go Left" and not self.goal_published:
            goal = MoveBaseGoal()
            self.goal_reached=True
            
            goal.target_pose.header.frame_id = "t265_odom_frame"
            goal.target_pose.header.stamp = rospy.Time.now() 
            goal.target_pose.pose = self.pose_right_seq[self.goal_cnt]
            rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
            rospy.loginfo(str(self.pose_right_seq[self.goal_cnt]))
            self.pose_seq=self.pose_right_seq
            self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
            # rospy.spin()
        elif msg.data=="A_Goal":
            self.goal_cnt=0
            self.goal_reached="A_Reached"
            self.pose_seq=self.pose_right_seq
            goal.target_pose.pose = self.pose_right_seq[self.goal_cnt]
            rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
            rospy.loginfo(str(self.pose_right_seq[self.goal_cnt]))
            self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
            # rospy.spin()
        elif msg.data=="B_Goal":
            self.goal_cnt=0
            self.goal_reached="B_Reached"
            self.pose_seq=self.pose_left_seq
            goal.target_pose.pose = self.pose_left_seq[self.goal_cnt]
            rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
            rospy.loginfo(str(self.pose_left_seq[self.goal_cnt]))
            self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
            # rospy.spin()
        elif msg.data=="A_Goal_Recovery":
            self.goal_reached="A_Reached"
            self.pose_seq=self.pose_right_seq
            self.goal_cnt=len(self.pose_seq)-1
            goal.target_pose.pose = self.pose_right_seq[self.goal_cnt]
            rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" as A_Goal_Recovery to Action Server")
            rospy.loginfo(str(self.pose_right_seq[self.goal_cnt]))
            self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
            # rospy.spin()
        elif msg.data=="B_Goal_Recovery":
            self.goal_reached="B_Reached"
            self.pose_seq=self.pose_left_seq
            self.goal_cnt=len(self.pose_seq)-1
            goal.target_pose.pose = self.pose_left_seq[self.goal_cnt]
            rospy.loginfo(str(self.pose_left_seq[self.goal_cnt]))
            rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+"as B_Goal_Recovery to Action Server")
            self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
        elif msg.data=="Tool_Goal":
            self.tool_flag=True
            self.goal_cnt=0
            self.pose_seq=self.pose_start_seq
            goal.target_pose.pose = self.pose_start_seq[self.goal_cnt]
            rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
            rospy.loginfo(str(self.pose_start_seq[self.goal_cnt]))
            self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)            
        elif msg.data=="Tool_Goal_Recovery":
            self.tool_flag=True
            self.pose_seq=self.pose_start_seq
            self.goal_cnt=1
            goal.target_pose.pose = self.pose_start_seq[self.goal_cnt]
            rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+"as Tool_Goal_Recovery  to Action Server")
            rospy.loginfo(str(self.pose_start_seq[self.goal_cnt]))
            self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
            # rospy.spin()


            
        self.goal_published = True
        
        
    def subscriber_node(self):
        # rospy.Subscriber('/predicted_result', String, self.movebase_client)
        # result = movebase_client()
        # result= self.movebase_client_single_pose(self)

        
        # rospy.Subscriber('/arm_node/staring_pose_feedback', Bool, self.movebase_client_single_pose)
        rospy.Subscriber('/main_node/goal_location', String, self.movebase_client)
        # rospy.Subscriber('/arm_node/hand_off_feedback', Bool, self.movebase_client)
        rospy.spin()
    

if __name__ == '__main__':
    try:
        rospy.init_node('multi_goal_navigation', anonymous=False)
        
        MoveBaseSeq()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")
        
        
        
        
        
        
        
        
