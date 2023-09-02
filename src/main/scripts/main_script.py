#!/usr/bin/env python3

# Master main script for communicate Ur5 arm robot, mobile base and GUI
# Author : Charlotte Wang
import os
import time
import rospy
from std_msgs.msg import String, Float32

os.environ['ROS_MASTER_URI'] = 'http://192.168.1.148:11311'

def task_list_format(urgent_task, normal_task):
    s = ""
    for i in urgent_task:
        s = s + i + ","
    
    for i in normal_task:
        s = s + i + ","

    return s

class ArmWithBase(object):
    def __init__(self):
        rospy.init_node("main_node")
        self.normal_task = []
        self.urgent_task = []
        #region Arm
        self.pickup_tool_pub = rospy.Publisher(
            '/main_node/pickup_tool',
            String,
            queue_size=1
        )
        self.pickup_feedback_sub = rospy.Subscriber(
            '/arm_node/pickup_feedback',
            String,
            self.pickup_feedback_cb
        )
        self.pickup_feedback = ""
        self.handoff_pub = rospy.Publisher(
            '/main_node/handoff_signal',
            String,
            queue_size=1
        )
        self.handoff_feedback_sub = rospy.Subscriber(
            '/arm_node/handoff_feedback',
            String,
            self.handoff_feedback_cb
        )
        self.handoff_feedback = ""
        self.reset_pub = rospy.Publisher(
            '/main_node/reset',
            String,
            queue_size=1
        )
        self.reset_feedback_sub = rospy.Subscriber(
            '/arm_node/reset_feedback',
            String,
            self.reset_feedback_cb
        )
        self.reset_feedback = ""
        #endregion

        #region Base
        self.goal_location_pub = rospy.Publisher(
            '/main_node/goal_location',
            String,
            queue_size=1
        )
        self.goal_achieved_sub = rospy.Subscriber(
            '/goal_achieved', 
            String, 
            self.goal_achieved_cb
        )
        self.goal_achieved = ""
        # self.estimated_time_sub = rospy.Subscriber(
        #     '/estimated_time',
        #     Float32,
        #     self.estimated_time_cb
        # )
        # self.estimated_time = ""
        #endregion
        
        #region GUI
        self.estimated_time_pub = rospy.Publisher(
            '/main_node/estimated_time',
            String,
            queue_size=1
        )
        self.tool_status_pub = rospy.Publisher(
            '/main_node/status',
            String,
            queue_size=1
        )
        self.task_list_pub = rospy.Publisher(
            '/main_node/task_list',
            String,
            queue_size=1
        )
        self.tool_request_sub = rospy.Subscriber(
            '/gui_node/tool_task',
            String,
            self.tool_request_cb
        )
        #endregion

    #region callback function
    def pickup_feedback_cb(self, msg):
        self.pickup_feedback = msg.data
    
    def handoff_feedback_cb(self, msg):
        self.handoff_feedback = msg.data
    
    def reset_feedback_cb(self, msg):
        self.reset_feedback = msg.data
    
    def goal_achieved_cb(self, msg):
        # print(msg.data)
        self.goal_achieved = msg.data
    
    def estimated_time_cb(self, msg):
        self.estimated_time = msg.data
        self.estimated_time_pub.publish(msg)

    def tool_request_cb(self, msg):
        request = msg.data
        if request != "":
            # print(request)
            if "Urgent" in request:
                self.urgent_task.append(request)
            else:
                self.normal_task.append(request)
            if len(self.urgent_task) != 0 and len(self.normal_task) != 0:
                self.task_list_pub.publish(task_list_format(self.urgent_task, self.normal_task))
    #endregion
        
    def pickup(self, target_tool, duration=5):
        print("Pick up: ", target_tool)
        self.pickup_tool_pub.publish(f"Scan_{target_tool}_{duration}")
        while True:
            while self.pickup_feedback == "":
                self.task_list_pub.publish(task_list_format(self.urgent_task, self.normal_task))
                time.sleep(0.1)
            
            s = self.pickup_feedback.split(" ")
            self.pickup_feedback = ""
            if s[0] == "Success":
                print("Pick up succeed.")
                return "Success"
            elif s[0] == "No_Target":
                print("No target to pick up.")
                return "No_Target"
            else:
                if s[1] != "CONTROL_FAILED":
                    print(f"Trying to: {s[0]}")
                    if s[0] == "Scan":
                        self.pickup_tool_pub.publish(f"Scan_{target_tool}_{duration}")
                    else:
                        self.pickup_tool_pub.publish(s[0])                    
                else:
                    print("Control Failed, check connection.")
                    if s[0] == "Scan":
                        self.pickup_tool_pub.publish(f"Scan_{target_tool}_{duration}")
                    else:
                        self.pickup_tool_pub.publish(s[0])
    
    def handoff(self, human_target, duration=2):
        print("Hand off to: ", human_target)
        self.handoff_pub.publish(f"Go_{human_target}_{duration}")
        while True:
            while self.handoff_feedback == "":
                self.task_list_pub.publish(task_list_format(self.urgent_task, self.normal_task))
                time.sleep(0.1)
            s = self.handoff_feedback.split(" ")
            self.handoff_feedback = ""
            if s[0] == "Success":
                print("Hand off succeed.")
                break
            elif s[0] == "NoTarget":
                self.handoff_pub.publish(f"Go_{human_target}_{duration}")
                print("No target detected, try hand off again.")
            else:
                if s[1] == "Scan":
                    self.handoff_pub.publish(f"Go_{human_target}_{duration}")
                elif s[1] != "CONTROL_FAILED":
                    self.handoff_pub.publish(s[0])
                    print("Trying to", s[0])
                else:
                    print("Control Failed, check connection.")
                    self.handoff_pub.publish(s[0])
    
    def goes_to(self, loc_target):
        print("Goes to: ", loc_target)
        self.goal_location_pub.publish(f"{loc_target}_Goal")
        while True:
            while self.goal_achieved == "":
                self.task_list_pub.publish(task_list_format(self.urgent_task, self.normal_task))
                time.sleep(0.1)
            
            s = self.goal_achieved
            print("In goes_to function: ", s)
            self.goal_achieved = ""
            if f"{loc_target}_Success" == s:
                print("Goal achieved at: ", loc_target)
                break
            elif "Aborted" == s:
                self.goal_location_pub.publish(f"{loc_target}_Goal_Recovery")
                print("Try again to go ", loc_target)
            
    def single_run(self, human, target_tool, human_target="Right"): # tool station(pick up) -> goes to human -> hand off -> tool station
        self.tool_status_pub.publish(f"Picking up: {target_tool}")
        self.estimated_time_pub.publish('')
        p = self.pickup(target_tool)
        if p == "Success":
            self.tool_status_pub.publish(f"Going to {human} station.")
            self.goes_to(human)
            # input('Human')

            self.tool_status_pub.publish(f"Hand off to {human}.")
            self.handoff(human_target)

            self.tool_status_pub.publish("Going to Tool station.")
            self.goes_to("Tool")
            # input('Tool')
            self.tool_status_pub.publish("Tool station achieved.")
            self.estimated_time_pub.publish('')
            time.sleep(1)
        elif p == "No_Target":
            self.tool_status_pub.publish(f"No {target_tool} available.")
            time.sleep(1)
        elif p == "Control_Failed":
            self.tool_status_pub.publish("Robot not available.")
            time.sleep(1)

    def reset(self, command):
        print("Reset: ", command)
        msg = String()
        msg.data = command
        self.reset_pub.publish(msg)
        while self.reset_feedback == "":
            time.sleep(0.1)
        
        self.reset_feedback = ""

    def countdown(self, human, target_tool, human_target="Right"): # For GUI panel recording
        self.tool_status_pub.publish(f"Picking up: {target_tool}")
        end_t = time.time() + 30
        c = 0
        t = 30
        while end_t - time.time() < 30:
            self.task_list_pub.publish(task_list_format(self.urgent_task, self.normal_task))
            time.sleep(0.1)
            c += 1
            if c == 10:
                self.estimated_time_pub.publish(str(t) + 's')
                t -= 1
                c = 0

    def task_handler(self):
        if len(self.urgent_task) != 0 or len(self.normal_task) != 0:
            print(self.urgent_task, self.normal_task)
            self.task_list_pub.publish(task_list_format(self.urgent_task, self.normal_task))

            if len(self.urgent_task) != 0:
                task = self.urgent_task.pop(0).split("_")
            elif len(self.normal_task) != 0:
                task = self.normal_task.pop(0).split("_")

            if task[0] == "A":
                self.estimated_time_pub.publish('31s')
            else:
                self.estimated_time_pub.publish('34s')
            # self.single_run(task[0], task[1])
            self.countdown(task[0], task[1])
        else:
            pass

if __name__ == "__main__":
    try:
        ab = ArmWithBase()
        while not rospy.is_shutdown():
            # ab.reset("Start")
            # ab.reset("Scan_center")
            # ab.reset(input("Reset: "))
            # ab.pickup(input("Tool: "))
            # ab.reset("Open")
            # ab.reset("Video_3")
            # ab.reset("Video_3.5")
            # ab.reset("Video_4")
            # ab.reset("Video_5")
            # ab.reset("Video_6")
            # ab.reset("Close")
            
            # ab.reset("Video_5")
            # ab.reset("Video_4")
            # ab.reset("Video_3.5")
            # ab.reset("Video_3")
            # ab.handoff("Right")
            # ab.goes_to(input("Goal:"))
            # ab.single_run("B", "Fasteners")
            # ab.single_run("B", "Hammer")
            # break
            ab.task_handler()
    except KeyboardInterrupt:
        rospy.loginfo("Interrupted")