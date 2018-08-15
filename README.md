# head_control

ROS node to control the head and synchronise a face recognition scan.

## Running the Node

Once you have the node built you can test it by launching the test.launch file.

## Node Information
Topics:

* `face_recognition_node/result`:  
  Subscribes `facerecognition_msgs/face_recognition` the result of the last image scan
  
* `face_recognition_node/start`:  
  Publishes `std_msgs/Empty` requests a face recognition scan of the next image
  
* `pan_tilt_node/head_position`:  
  Publishes `servo_msgs/pan_tilt` sets the servo position for the pan and tilt servo

Actions:

* `face_recognition_msgs/scan_for_faces`:  
  Server used to controls the moving of the head/camera and scanning for a recognised face at each set position.  
  
Parameters:

* `/servo/index0/pan_min`: minimum range for the pan servo. Default value = 0. 
* `/servo/index0/pan_max`: Maximum range for the pan servo. Default value = 180.
* `/servo/index0/tilt_min`: Minimum range for the tilt servo. Default value = 0.
* `/servo/index0/tilt_max`: Maximum range for the tilt servo. Default value = 180.
* `/head/view_step/pan`: The increment that the pan servo will move for each scan. Default value = 10.
* `/head/view_step/tilt`: The increment that the tile servo will move for each scan. Default value = 10.
* `/head/max_step/pan`: The maximum angle the pan servo will move in one go. Deafult value = 10.
* `/head/max_step/tilt`: The maximum angle the tilt servo will move in one go. Deafult value = 10.
* `/head/position/pan`: The pan position that the head will move to when a task is complete. Default value = 90.
* `/head/position/tilt`: The tilt position that the head will move to when a task is complete. Default value = 45.
