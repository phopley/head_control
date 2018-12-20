#ifndef HEAD_CONTROL_NODE_H_
#define HEAD_CONTROL_NODE_H_

#include <ros/ros.h>
#include <servo_msgs/pan_tilt.h>
#include <std_msgs/String.h>
#include <head_control/point_headAction.h>
#include <head_control/point_headActionFeedback.h>
#include <head_control/point_headActionResult.h>
#include <actionlib/server/simple_action_server.h>

typedef actionlib::SimpleActionServer<head_control::point_headAction> Server;

class HeadControlNode
{
protected:
    Server as_;
    std::string action_name_;

public:
    HeadControlNode(ros::NodeHandle n, std::string name);
    ~HeadControlNode();
    // Function to move the servos
    void moveServo();

private:
    ros::NodeHandle nh_;
		
    servo_msgs::pan_tilt current_pan_tilt_; // The current pan/tilt position
    servo_msgs::pan_tilt target_pan_tilt_;  // The position we want the pan/tilt to move to
    servo_msgs::pan_tilt default_position_; // Neutral position for pan and tilt
		
    ros::Publisher move_head_pub_;
		
    int pan_step_;              // Maximum angle we can move the pan servo in one go 
    int tilt_step_;             // Maximum angle we can move the tilt servo in one go
    
    int loop_count_down_;       // Used to give time for head to settle before stating movement complete
    
    bool move_head_;            // Set to true when servos should move to target position
    bool movement_complete_;    // Set to true when servo movement complete and next process requested

    // This callback is for the point head action
    void pointHeadCallback();
};

#endif // HEAD_CONTROL_NODE_H_
