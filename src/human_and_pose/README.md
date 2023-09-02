***Human and hand detection for ROS***

In this package, we using YOLO model from Roboflow for human detection and mediapipe for detecting hand gesture. 

It will publish three message after detection: 
1. "/human_hand_node/human_info"
This contain a Float32MultiArray which have the nearest human's coordinate to the camera
[human.x, human.y, human.z]

2. "/human_hand_node/hand_info"
This message containing the left and right hand infomation detected from mediapipe. According to mediapipe, we will have 21 points' coordinates for eacch finger joint in one hand. Then we publish as PointCloud2 with six fields: right_x, right_y, right_z, left_x, left_y, left_z

3. "/human_hand_node/completed"
This message is used for sending a signal to the main process which needs to know the status of human and hand detection.