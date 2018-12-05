#include <ros/ros.h>
#include <head_control/head_control_node.h>
#include <std_msgs/Empty.h>


// This callback is used to process a request to move the head
void HeadControlNode::requestedMovementCallback(const servo_msgs::pan_tilt& msg)
{
    // Set the target position to the request position 
    target_pan_tilt_ = msg;

    // Indicate that the servos should be moved
    move_head_ = true;
    
    // If we just reach the position and a new message arrives we need to have movement complete false
    movement_complete_ = false;
}
//---------------------------------------------------------------------------

// Function to move the servos if required by a step amount. This is to stop the head shuddering if the servo
// is moved to the target position in one movement.
void HeadControlNode::moveServo()
{
    if(move_head_ == true)
    {
        // Still processing servo movement
        if(movement_complete_ == true)
        {
            // We have reached the target but give time to settle
            loop_count_down_--;
            
            if(loop_count_down_ <= 0)
            {
                movement_complete_ = false;
                move_head_ = false;                
                
                // Send movement complete message
                std_msgs::Empty mt;
                move_complete_pub_.publish(mt);                
            }
        }
        else
        {
            if((target_pan_tilt_.pan == current_pan_tilt_.pan) && (target_pan_tilt_.tilt == current_pan_tilt_.tilt))
            {
                // Last time around we must have requested the final move
                movement_complete_ = true;
                loop_count_down_ = 8;
            }        
            else
            {                
                // Still moving, calculate pan movement
                if(std::abs(target_pan_tilt_.pan - current_pan_tilt_.pan) > pan_step_)
                {
                    // Distance to target to great to move in one go
                    if(target_pan_tilt_.pan > current_pan_tilt_.pan)
                    {
                        // Add the step to current
                        current_pan_tilt_.pan += pan_step_;
                    }
                    else
                    {
                        // Subtract step from current 
                        current_pan_tilt_.pan -= pan_step_;
                     }
                }
                else 
                {
                    // Can move to the target position in one go (or pan is in fact already there but tilt is not)
                    current_pan_tilt_.pan = target_pan_tilt_.pan;                                                            
                }
        
                // Calculate tilt movement
                if(std::abs(target_pan_tilt_.tilt - current_pan_tilt_.tilt) > tilt_step_)
                {
                    // Distance to target to great to move in one go
                    if(target_pan_tilt_.tilt > current_pan_tilt_.tilt)
                    {
                        // Add the step to current
                        current_pan_tilt_.tilt += tilt_step_;
                    }
                    else
                    {
                        // Subtract step from current 
                        current_pan_tilt_.tilt -= tilt_step_;
                     }
                }
                else 
                {
                    // Can move to the target position in one go (or tilt is in fact already there but pan is not)
                    current_pan_tilt_.tilt = target_pan_tilt_.tilt;                                                            
                }
                
                
                // Publish the movement
                move_head_pub_.publish(current_pan_tilt_);
            }
        }
    }
}

// Constructor 
HeadControlNode::HeadControlNode(ros::NodeHandle n)
{	
    nh_ = n;
	
    // Subscribe to topic for head movement request
    movement_req_sub_ = 
        nh_.subscribe("/head_control_node/head_position_request", 5, &HeadControlNode::requestedMovementCallback, this);          
		    	
    // Topic to move head
    move_head_pub_ = nh_.advertise<servo_msgs::pan_tilt>("pan_tilt_node/head_position", 10);
    
    // Topic to indicate movement complete
    move_complete_pub_ = nh_.advertise<std_msgs::Empty>("/head_control_node/head_move_complete", 5);
	
    pan_step_ = 10;     // Maximum step movment for pan servo 
    tilt_step_ = 10;    // Maximum step movment for tilt servo 
	
    // Obtain any configuration values from the parameter server. If they don't exist use the defaults above
    nh_.param("/head/max_step/pan", pan_step_, pan_step_);
    nh_.param("/head/max_step/tilt", tilt_step_, tilt_step_);
        
    int pan = 90;      // Pan default position to return to
    int tilt = 45;     // Tilt default position to return to
    nh_.param("/head/position/pan", pan, pan);    
    nh_.param("/head/position/tilt", tilt, tilt);
    default_position_.pan = (int)pan;
    default_position_.tilt = (int)tilt;
	
    // This start position may be so the user can access the on screen keyboard. 
    // We will often return to this position when a task is completed	
    current_pan_tilt_ = default_position_;
    // We don't know where the servo starts from so just jump to the required position    
    // Publish a start position to get the head in a known position.
    move_head_pub_.publish(current_pan_tilt_);
    
    move_head_ = false;
    movement_complete_ = false;
    target_pan_tilt_ = current_pan_tilt_;
}
//---------------------------------------------------------------------------

// Destructor
HeadControlNode::~HeadControlNode()
{
}
//---------------------------------------------------------------------------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "head_control_node");
    ros::NodeHandle n;
    std::string node_name = ros::this_node::getName();
    HeadControlNode head_control(n);	
    ROS_INFO("%s started", node_name.c_str());
    
    // We need control of the node so that we can step the servos to the target 
    // position in small steps to stop the head shuddering if it was to move in one big step
    
    ros::Rate r(10); // 10Hz
    
    while(ros::ok())
    {
        // See if the servos need moving
        head_control.moveServo();
        
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}

