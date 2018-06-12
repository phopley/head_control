# head_control
ROS package for the head control node.
## Description
The node controls the movment of the robot head and synchronised the request for face recognition with the head movment.
## Executable
head_control_node
## Subscribe topics
- /face_recognition_node/result of type face_recognition_msgs.face_recognition
## publishes topics 
- face_recognition_node/start of type std_msgs.Empty
- pan_tilt_node/head_position of type servo_msgs.pan_tilt
## Fixed server parameters
- /servo/index0/pan_min, default = 0, minimum range for the pan servo
- /servo/index0/pan_max, default = 180, maximum range for the pan servo
- /servo/index0/tilt_min, default = 0, minimum range for the tilt servo
- /servo/index0/tilt_max, default = 180, maximum range for the tilt servo
- /head/step/pan, default = 10, the increment that the pan will move for each scan
- /head/step/tilt, default = 10, the increment that the tilt will move for each scan
- /head/position/pan, default = 90, the pan position that the head will move to when a task is complete
- /head/position/tilt, default = 45, the tilt position that the head will move to when a task is complete

