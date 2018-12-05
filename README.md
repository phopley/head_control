# head_control

ROS node to control the head/camera by controlling the speed of the servos to reach a target position.

## Running the Node

Once you have the node built you can test it by launching the test.launch file.

## Node Information
Topics:

* `/head_control_node/head_position_request`:
  Subscribes `servo_msgs::pan_tilt` The target position to command the pan tilt servo to 

* `pan_tilt_node/head_position`:  
  Publishes `servo_msgs/pan_tilt` sets the servo position for the pan and tilt servo
  
* `/head_control_node/head_move_complete`:
  Publishes `std_msgs/Empty` indicates the servos have been commanded to the final target position  
  
Parameters:

* `/head/max_step/pan`: The maximum angle the pan servo will move in one go. Deafult value = 10.
* `/head/max_step/tilt`: The maximum angle the tilt servo will move in one go. Deafult value = 10.
* `/head/position/pan`: The default pan position that the head will move to. Default value = 90.
* `/head/position/tilt`: The default tilt position that the head will move to. Default value = 45.
