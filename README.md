# head_control

ROS node to control the head/camera by controlling the speed of the servos to reach a target position.

## Running the Node

Once you have the node built you can test it by launching the test.launch file.

## Node Information
Topics:

* `pan_tilt_node/joints`:  
  Publishes `sensor_msgs/JointState` sets the position for the pan and tilt joints

Action:

* `head_control/point_head`:  
  Server used to control the process of moving the head to the target position.
  
Parameters:

* `/servo/index0/pan/joint_name`: Name of the pan joint. Default = "reserved_pan0"
* `/servo/index0/tilt/joint_name`: Name of the tilt joint. Default = "reserved_tilt0"
* `~head/max_step/pan`: The maximum angle the pan servo will move in one go. Deafult value = 0.174533 radians.
* `~head/max_step/tilt`: The maximum angle the tilt servo will move in one go. Deafult value = 0.174533 radians.

## License
Software source and object files are licensed under the Apache License, Version 2.0. See the License for the specific language governing permissions and limitations under the License.
