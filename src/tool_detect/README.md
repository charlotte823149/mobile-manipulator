***Tool detection fo ROS***

In this package we use YOLO on Roboflow to detection basic mechanical tools. 

First, it will waiting for a start signal then detect the target tool get from the main script. After detection, it will crop out the region and publish to the topic that simple_grasping is subscribed to. With simple_grasping segment out the object, it publish to graspable_tool.

1. "tool_detect_node/detected_tool"
detected_tool PointCloud2 data is only for simple_grasping to make the further movement.

2. "tool_detect_node/graspable_tool"
After simple_grasping segment out the support surface and objects, it publish out to graspable_tool topic

3. "tool_detect_node/completed"
Sending the completed signal for main script to know the status of the tool detection.