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
    HeadControlNode(ros::NodeHandle n, std::string name);
    ~HeadControlNode();
    // Function to move the servos
    void moveServo();

private:
    ros::NodeHandle nh_;
		
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
