#!/usr/bin/env python3

# Combining simple grasping with roboflow api model
# Author : Charlotte Wang

import cv2
import json
import time
import rospy
import datetime
import actionlib
import message_filters
import sensor_msgs.point_cloud2 as pc2

from ultralytics import YOLO
from roboflow import Roboflow
from cv_bridge import CvBridge
from std_msgs.msg import String, Bool
from colorama import Fore, Back, Style
from sensor_msgs.msg import CameraInfo, PointCloud2, Image as ImageSensor_msg
from grasping_msgs.msg import FindGraspableObjectsResult, FindGraspableObjectsAction, FindGraspableObjectsGoal

class ToolDetectNode(object):
    def __init__(self):
        rospy.init_node('tool_detect_node')
        self.cv_bridge = CvBridge()
        self.target_tool = ""
        self.start_time = 0.0
        self.surface = 0
        self.tool = 0
        self.along_axis = ""

        # train6 -> tools, train7 -> tools + fasteners
        self.model = YOLO('/home/cam/Tool_detection/runs/detect/train7/weights/best.pt')

        self.find_client = actionlib.SimpleActionClient("basic_grasping_perception/find_objects", FindGraspableObjectsAction)
        self.find_client.wait_for_server()

        # ROS Subscriber
        image_sub = message_filters.Subscriber(
            "/camera/color/image_raw",
            ImageSensor_msg
        )        
        depth_sub = message_filters.Subscriber(
            "/camera/depth/color/points",
            PointCloud2
        )
        info_sub = message_filters.Subscriber(
            "/camera/color/camera_info",
            CameraInfo
        )
        self.target_tool_sub = rospy.Subscriber(
            '/arm_node/target_tool', 
            String, 
            self.target_tool_cb
        )         

        # ROS Publishers
        self.detected_tool_pub = rospy.Publisher(
            '~detected_tool',
            PointCloud2,
            queue_size=1
        )
        self.graspable_tool_pub = rospy.Publisher(
            '~graspable_tool',
            FindGraspableObjectsResult,
            queue_size=1
        )
        self.tool_axis_pub = rospy.Publisher(
            '~tool_axis',
            String,
            queue_size=1
        )
        self.complete_pub = rospy.Publisher(
            '~completed',
            Bool,
            queue_size=1
        )

        ts = message_filters.TimeSynchronizer([image_sub, depth_sub, info_sub], 1)
        ts.registerCallback(self.image_cb)
        rospy.loginfo("Ready for Tool detection")
        rospy.loginfo("Waiting for signal......")

    def image_cb(self, image_msg, depth_msg, camera_info):
        if self.target_tool != "":
            print("Received sufficient info, start detecting: ", self.target_tool)
            img = self.cv_bridge.imgmsg_to_cv2(image_msg, "rgb8")
            w = camera_info.width
            h = camera_info.height
            fx = camera_info.K[0]
            fy = camera_info.K[4]
            cx = camera_info.K[2]
            cy = camera_info.K[5]

            results = self.model(img, conf=0.4, verbose=False)
            j = []
            j = json.loads(results[0].tojson())

            pixel_range = []
            flag = False
            confidence = 0.0
            print("Detecting tool: ")

            for obj in j:
                if 'box' in obj:
                    x_min = int(obj['box']['x1'])
                    x_max = int(obj['box']['x2'])
                    y_min = int(obj['box']['y1'])
                    y_max = int(obj['box']['y2'])

                    img_w = x_max - x_min + 80
                    img_h = y_max - y_min + 80

                    x_min = int(x_min - 40)
                    if x_min < 0.0: x_min = int(0.0)
                    x_max = int(x_max + 40)
                    if x_max > w: x_max = int(w)
                    y_min = int(y_min - 40)
                    if y_min < 0.0: y_min = int(0.0)
                    y_max = int(y_max + 40)
                    if y_max > h: y_max = int(h)

                    if self.target_tool == obj['name'] and obj['confidence'] > confidence:
                        confidence = obj['confidence']

                        if self.target_tool == "Fasteners": self.along_axis = "x"
                        elif img_w > img_h: self.along_axis = "y"
                        else: self.along_axis = "x"
                                
                        pixel_range = [x_min, x_max, y_min, y_max]
                        cv2.circle(img, (int(x_min + img_w/2), int(y_min + img_h/2)), 5, (255, 0, 0), 5)
                        color = (255, 0, 0)
                        
                        print(Fore.GREEN + obj['name'] + Style.RESET_ALL)
                        flag = True
                    else:
                        color = (255, 255, 255)
                        print(obj['name'])

                cv2.rectangle(img, (x_min, y_min), (x_max, y_max), color, 2)
                cv2.putText(img, f"{obj['name']}: {obj['confidence']:.2f}", (x_min - 4, y_min - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                
                current_time = datetime.datetime.now()
                formatted_time = current_time.strftime("%Y-%m-%d_%H-%M-%S")
                cv2.imwrite(f'/home/cam/charlotte_ws/src/tool_detect/image/{formatted_time}.png', cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
            
            with open('/home/cam/charlotte_ws/src/tool_detect/tool_detect.json', 'w') as f:
                json.dump({"predictions": j}, f, indent=4)
            
            if flag:
                pc_data = pc2.read_points(depth_msg, field_names=("x", "y", "z", "rgb"), skip_nans=True)
                filtered_points = []
                for point in pc_data:
                    x, y, z, rgb = point
                    pixel_x = int(fx * x/z + cx)
                    pixel_y = int(fy * y/z + cy)
                    
                    if pixel_range[0] <= pixel_x <= pixel_range[1] and pixel_range[2] <= pixel_y <= pixel_range[3]:
                        filtered_points.append(point)
            
                header = depth_msg.header
                fields = [
                    pc2.PointField(name="x", offset=0, datatype=pc2.PointField.FLOAT32, count=1),
                    pc2.PointField(name="y", offset=4, datatype=pc2.PointField.FLOAT32, count=1),
                    pc2.PointField(name="z", offset=8, datatype=pc2.PointField.FLOAT32, count=1),
                    pc2.PointField(name="rgb", offset=16, datatype=pc2.PointField.FLOAT32, count=1)
                ]
                point_cloud_msg = pc2.create_cloud(header, fields, filtered_points)

                self.detected_tool_pub.publish(point_cloud_msg)
                
                goal = FindGraspableObjectsGoal()
                self.find_client.send_goal(goal)
                self.find_client.wait_for_result()
                self.find_result = self.find_client.get_result()
                self.surface = len(self.find_result.support_surfaces)
                self.tool = len(self.find_result.objects)
                self.detection_check(flag)
    
    def target_tool_cb(self, msg):
        self.target_tool = msg.data
        if self.target_tool != "":
            self.start_time = time.time()
            rospy.loginfo(f"Detecting target: {self.target_tool}")
        else:
            rospy.loginfo("No target tool, detection stop")

    def detection_check(self, flag:bool):
        print(Fore.YELLOW + f"Numbers of detected support surface: {self.surface}")
        print(f"Numbers of detected object: {self.tool}" + Style.RESET_ALL)

        if flag and self.tool != 0:
            result_msg = self.find_result
            self.graspable_tool_pub.publish(result_msg)
            complete_msg = Bool()
            complete_msg.data = True
            self.complete_pub.publish(complete_msg)
            axis_msg = String()
            axis_msg.data = self.along_axis
            self.tool_axis_pub.publish(axis_msg)
            rospy.loginfo("Tool detection completed")       

def main():
    try:
        ToolDetectNode()
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
