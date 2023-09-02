# Project: mobile-manipulator
This project is the document for Ur5 arm cooperate with mobile base to implement an robot assistant system. Aim to free human worker from tideous and repetitive movement from getting mechanical tools. Base on ROS and realsence camera, we used three computers for handling the system, which is the master script(this repository), GUI and mobile base. In this repository, we have 1. master main script for handling the comminucation of arm, GUI and mobile base 2. arm robot motion planning 3. yoloV8 for mechanical tool detection 4. yoloV8 for human instance detection and utilize mediaipe for hand pose detection.

## require packages
For starting this project you will need some packages from other repositories.
- Universal robot: https://github.com/ros-industrial/robotiq
- realsense camera: https://github.com/IntelRealSense/librealsense
- simple grasping: https://github.com/mikeferguson/simple_grasping
- mediapipe(place under tool_detect/src): https://github.com/google/mediapipe

## require hardwares(for this repository)
Ur5 arm robot
realsense camera d415 or advanced version x2

## tool_detect
### src
tool.detect.py:
In this project, we are using yoloV8 self-trained model for five categories bounding box detection, which are: Hammer, Wrench, Pliers, Screwdriver and Fasteners box(a black solid box with bolts inside). After the getting the pixel range of the target object, the script will further publish the point cloud data within this range to let simple grasping segment out the object's outline. With object's pose and orientation, we can further calculate the desire grasping position on the object.

## human_and_pose
### src
human_and_hand.py:
For implement a secured hand off, we use yoloV8 pre_trained model to getting the bounding box infomation of human body. Then use this pixel range as an input for mediapipe hand pose detection, which can cut down the computation cost for the image processing. Meanwhile, a realsense camera d415 will record the depth value for each point cloud data, which will further level the accuracy of hand off distance.

## main
### launch
robot.launch:
Launch sufficient package for establish the communication with Ur5 robot and moveit for commanding robot.

tool.launch:
Launch realsnse camera, simple grasping and tool_detect script.

human.launch:
Launch human and hand gesture detection script.

### scripts
main_script.py:
1. Handling the communication of arm robot and mobile base.
2. Display the sufficient information to GUI window, such as: tool status, estimated time and task queue list.
3. Handling the "urgent task" and "regular task" process order.

with_base.py:
1. Ur5 arm robot motion planning.
2. Setting basic pose movement, like: startting, scaning and tool-in-hand posture.
3. Communication with tool detection script: sending start and end signal, processing result publish from simple grasping.
4. Communication with human and hand pose detection script: sendign start and end signal, transfering result from camera coordinate to robot's coordinate.
