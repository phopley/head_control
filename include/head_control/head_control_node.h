/* Copyright 2019 Philip Hopley
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not 
 * use this file except in compliance with the License. You may obtain a  copy
 * of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software 
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 *
 * ROS node to control the head/camera by controlling the speed of the servos to reach a target position. 
 */
#ifndef HEAD_CONTROL_NODE_H_
#define HEAD_CONTROL_NODE_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <head_control/point_headAction.h>
#include <head_control/point_headActionFeedback.h>
#include <head_control/point_headActionResult.h>
#include <actionlib/server/simple_action_server.h>

typedef actionlib::SimpleActionServer<head_control::point_headAction> Server;

struct position
{
    double pan;
    double tilt;
};

class HeadControlNode
{
protected:
    Server as_;

public:
    HeadControlNode(ros::NodeHandle n, ros::NodeHandle n_private, std::string name);
    ~HeadControlNode();
    // Function to move the servos
    void moveServo();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
		
    position current_pan_tilt_; // The current pan/tilt position
    position target_pan_tilt_;  // The position we want the pan/tilt to move to
    position default_position_; // Neutral position for pan and tilt
		
    ros::Publisher move_head_pub_;
		
    double pan_step_;           // Maximum angle, in radians, we can move the pan servo in one go 
    double tilt_step_;          // Maximum angle, in radians, we can move the tilt servo in one go
    
    int loop_count_down_;       // Used to give time for head to settle before stating movement complete
    
    bool move_head_;            // Set to true when servos should move to target position
    bool movement_complete_;    // Set to true when servo movement complete and next process requested

    std::string pan_joint_name_;    // Name of the pan joint
    std::string tilt_joint_name_;   // Name of the tilt joint

    sensor_msgs::JointState msg_;   // Message to publish

    void pointHeadCallback();                  // This callback is for the point head action
    void publishJointState(struct position);   // Fill out and send the joint state message
};

#endif // HEAD_CONTROL_NODE_H_
