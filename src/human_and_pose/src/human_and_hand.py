#!/usr/bin/env python

# Human tracking and hand gesture detection
# Author : Charlotte Wang

import os
import cv2
import time
import rospy
import json
import struct
import datetime
import numpy as np
import message_filters
import mediapipe as mp
import pyrealsense2 as rs

from ultralytics import YOLO
from roboflow import Roboflow
from colorama import Fore, Back, Style
from std_msgs.msg import Bool, Float32MultiArray, Float32
from sensor_msgs.msg import PointCloud2, PointField, CameraInfo, Image as ImageSensor_msg

def create_point_cloud(right_x, right_y, right_z, left_x, left_y, left_z):
    # Create PointCloud2 message
    msg = PointCloud2()

    # Set header
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "base_link"  # Set the frame ID

    # Define fields for right-hand coordinates
    righthand_fields = [
        PointField(name="right_x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="right_y", offset=4, datatype=PointField.FLOAT32, count=1), # 84 = 21 data * 8 bytes
        PointField(name="right_z", offset=8, datatype=PointField.FLOAT32, count=1), # the 84 bytes after y
    ]
    lefthand_fields = [
        PointField(name="left_x", offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name="left_y", offset=16, datatype=PointField.FLOAT32, count=1), # 84 = 21 data * 8 bytes
        PointField(name="left_z", offset=20, datatype=PointField.FLOAT32, count=1), # the 84 bytes after y
    ]

    # Combine the fields
    msg.fields = righthand_fields + lefthand_fields

    # Set height and width
    msg.height = 1
    msg.width = len(right_x)

    # Calculate the total size of one point
    msg.point_step = 24  # 6 fields * 4 bytes (float32)

    # Set the data buffer size
    msg.row_step = msg.point_step * msg.width
    msg.data = bytearray(msg.row_step * msg.height)

    # Iterate over the points and fill in the data buffer
    for i in range(len(right_x)):
        idx = i * msg.point_step
        struct.pack_into("ffffff", msg.data, idx, right_x[i], right_y[i], right_z[i], left_x[i], left_y[i], left_z[i])

    return msg

class HumanHandDetectNode:
    def __init__(self):
        rospy.init_node('human_hand_node')
        self.detect_time = 0.0
        self.start_time = 0.0
        self.draw = True
        self.frame = None
        self.image = None

        self.model = YOLO('/home/cam/Tool_detection/yolov8n.pt')

        # Initialize mediapipe
        self.mp_holistic = mp.solutions.holistic
        self.holistic = self.mp_holistic.Holistic(min_detection_confidence=0.3, min_tracking_confidence=0.3)
        self.mp_drawing = mp.solutions.drawing_utils

        # Configure Intel RealSense pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        serial_number = '840412060409'
        rospy.loginfo(f"Start streaming from device: {serial_number}")
        self.config.enable_device(serial_number)
        self.config.enable_stream(rs.stream.depth, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, rs.format.rgb8, 30)
        self.pipeline.start(self.config)
        self.intrinsics = rs.intrinsics()

        # ROS Subscriber
        self.detect_time_sub = rospy.Subscriber(
            '/arm_node/human_hand_detect', 
            Float32, 
            self.detect_cb
        )
        
        # ROS Publisher
        self.human_pub = rospy.Publisher(
            '~human_info', 
            Float32MultiArray, 
            queue_size=10
        )        
        self.hand_pub = rospy.Publisher(
            '~hand_info', 
            PointCloud2, 
            queue_size=1
        )
        self.completed_pub = rospy.Publisher(
            '~completed',
            Bool,
            queue_size=1
        )        
        
        rospy.loginfo("Starting Human and Hand detection")

    def detect_cb(self, msg):
        self.detect_time = msg.data
        if self.detect_time != 0.0:
            self.start_time = time.time()
        rospy.loginfo("Received signal: " + str(self.detect_time))
    
    def get_frames(self):
        # Read the frame from the camera
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        self.depth_frame = frames.get_depth_frame()

        # Color frame - get frame image
        frame = np.asanyarray(color_frame.get_data())
        self.frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        self.image = self.frame.copy()

        # Depth_frame - Access intrinsic properties
        depth_stream = self.depth_frame.profile.as_video_stream_profile().intrinsics
        self.intrinsics.fx = depth_stream.fx
        self.intrinsics.fy = depth_stream.fy
        self.intrinsics.ppx = depth_stream.ppx
        self.intrinsics.ppy = depth_stream.ppy
        self.intrinsics.height, self.intrinsics.width = frame.shape[:2]

    def process_hand_landmarks(self, landmarks):
        x = []
        y = []
        depth = []
        for l in landmarks:            
            p_x = l.x * self.intrinsics.width
            p_y = l.y * self.intrinsics.height
            if p_x < 0: p_x = 0
            if p_x > self.intrinsics.width: p_x = self.intrinsics.width - 1
            if p_y < 0: p_y = 0
            if p_y > self.intrinsics.height: p_y = self.intrinsics.height - 1
            
            d = self.depth_frame.get_distance(int(p_x), int(p_y))
            if d > 1.5: d = 1.5
            p = rs.rs2_deproject_pixel_to_point(self.intrinsics, [p_x, p_y], d)

            x.append(p[0])
            y.append(p[1])
            depth.append(d)
        return x, y, depth

    def hand_detection(self):
        results = self.holistic.process(self.frame)

        # Right hand detection
        if results.right_hand_landmarks != None:
            right_x, right_y, right_depth = self.process_hand_landmarks(results.right_hand_landmarks.landmark)
            print(Fore.BLUE + f"Right Hand: {right_x[12]:.2f}, {right_y[12]:.2f}, {right_depth[12]:.2f}" + Style.RESET_ALL)
        else:
            right_x = right_y = right_depth = [0.0] * 21
            print("No Right Hand")
        
        # Left hand detection
        if results.left_hand_landmarks != None:
            left_x, left_y, left_depth = self.process_hand_landmarks(results.left_hand_landmarks.landmark)
            print(Fore.GREEN + f"Left Hand: {left_x[12]:.2f}, {left_y[12]:.2f}, {left_depth[12]:.2f}\n" + Style.RESET_ALL)
        else:
            left_x = left_y = left_depth = [0.0] * 21
            print("No Left Hand\n")

        if self.draw:
            self.mp_drawing.draw_landmarks(self.image, results.right_hand_landmarks, self.mp_holistic.HAND_CONNECTIONS, 
                                        self.mp_drawing.DrawingSpec(color=(80,22,10), thickness=2, circle_radius=4),
                                        self.mp_drawing.DrawingSpec(color=(80,44,121), thickness=2, circle_radius=2))
            self.mp_drawing.draw_landmarks(self.image, results.left_hand_landmarks, self.mp_holistic.HAND_CONNECTIONS, 
                                        self.mp_drawing.DrawingSpec(color=(121,22,76), thickness=2, circle_radius=4),
                                        self.mp_drawing.DrawingSpec(color=(121,44,250), thickness=2, circle_radius=2))
            
        return create_point_cloud(right_x, right_y, right_depth, left_x, left_y, left_depth)

    def human_tracking(self):
        human = [0.0, 0.0, 0.0]
        flag = False

        results = self.model(self.frame, conf=0.4, verbose=False)
        j = []
        j = json.loads(results[0].tojson())
        confidence = float('-inf')
        depth = float('inf')
        for obj in j:
            if 'box' in obj:
                x_min = int(obj['box']['x1'])
                x_max = int(obj['box']['x2'])
                y_min = int(obj['box']['y1'])
                y_max = int(obj['box']['y2'])

                img_w = x_max - x_min + 80
                img_h = y_max - y_min + 80

                center_x = int(x_min + img_w/2)
                center_y = int(y_min + img_h/2)

                x_min = int(x_min - 40)
                if x_min < 0.0: x_min = int(0.0)
                x_max = int(x_max + 40)
                if x_max > self.intrinsics.width: x_max = int(self.intrinsics.width)
                y_min = int(y_min - 40)
                if y_min < 0.0: y_min = int(0.0)
                y_max = int(y_max + 40)
                if y_max > self.intrinsics.height: y_max = int(self.intrinsics.height)
                
                try:
                    # modify depth value to avoid noicy point clooud data
                    d = self.depth_frame.get_distance(center_x, center_y)
                    if d > 1.5: d = 1.5
                except RuntimeError as e:
                    print(e)
                    d = float('inf')
                if obj['name'] == "person" and obj['confidence'] > confidence and d < depth:
                    confidence = obj['confidence']
                    depth = d
                    p = rs.rs2_deproject_pixel_to_point(self.intrinsics, [center_x, center_y], depth)
                    human = [p[0], p[1], depth]

                    # Using human detection bounding box as an input for detecting hand pose
                    pixel_range = [y_min, y_max, x_min, x_max]
                    cv2.circle(self.image, (int(x_min + img_w/2), int(y_min + img_h/2)), 5, (255, 0, 0), 5)
                    color = (255, 0, 0)
                    flag = True
                else:
                    color = (255, 255, 255)

            cv2.rectangle(self.image, (x_min, y_min), (x_max, y_max), color, 2)
            cv2.putText(self.image, f"{obj['name']}: {obj['confidence']:.2f}", (x_min - 4, y_min - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        with open('/home/cam/charlotte_ws/src/human_and_pose/predict.json', 'w') as f:
            json.dump({"predictions": j}, f, indent=4)
        
        if flag:
            print(Fore.YELLOW + f"Human center: {human[0]:.2f}, {human[1]:.2f}, {human[2]:.2f}" + Style.RESET_ALL)
            
            new_f = np.zeros_like(self.frame)
            new_f[pixel_range[0]:pixel_range[1], pixel_range[2]:pixel_range[3]] = self.frame[pixel_range[0]:pixel_range[1], pixel_range[2]:pixel_range[3]]
            self.frame = new_f
        else:
            print("No Human")
        
        return human

    def detect_human_hand(self):
        if self.detect_time > 0.0:
            self.get_frames()
            human = self.human_tracking()
            hand_msg = self.hand_detection()
            human_msg = Float32MultiArray()
            human_msg.data = human
            self.human_pub.publish(human_msg)
            self.hand_pub.publish(hand_msg)
            rospy.sleep(0.1)
            if time.time() - self.start_time < self.detect_time:                
                current_time = datetime.datetime.now()
                formatted_time = current_time.strftime("%Y-%m-%d_%H-%M-%S")
                cv2.imwrite(f'/home/cam/charlotte_ws/src/human_and_pose/image/{formatted_time}.png', self.image)

def detector():
    try:
        human_hand = HumanHandDetectNode()
        # Try get the first frame to check the camera pipeline connection
        human_hand.get_frames()
        rospy.loginfo("Connection succeed, Waiting for signal...")

        while not rospy.is_shutdown():
            human_hand.detect_human_hand()

    except KeyboardInterrupt:
        rospy.loginfo("Interrupted")

if __name__ == '__main__':
    detector()