#include <ros/ros.h>
#include <head_control/head_control_node.h>


// Function to move the servos if required by a step amount. This is to stop the head shuddering if the servo
// is moved to the target position in one movement.
void HeadControlNode::moveServo()
{
    if(move_head_ == true)
    {
		if(as_.isPreemptRequested() || !ros::ok())
		{
			as_.setPreempted();

			movement_complete_ = false;
            move_head_ = false;
        }
        else if(movement_complete_ == true)
        {
            // We have reached the target but give time to settle
            loop_count_down_--;
            
            if(loop_count_down_ <= 0)
            {
                movement_complete_ = false;
                move_head_ = false;                                

				head_control::point_headResult result;
				as_.setSucceeded(result);
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
                publishJointState(current_pan_tilt_);

				// Publish feedback
				head_control::point_headFeedback feedback;
				as_.publishFeedback(feedback);
            }
        }
    }
}
//---------------------------------------------------------------------------

// This callback is for the point head action
void HeadControlNode::pointHeadCallback()
{
	head_control::point_headGoal::ConstPtr goal;

	goal = as_.acceptNewGoal();

    // Set the target position to the request position
    if (goal->absolute == true)
    {
        target_pan_tilt_.pan = goal->pan;
	    target_pan_tilt_.tilt = goal->tilt;
    }
    else
    {
        target_pan_tilt_.pan += goal->pan;
	    target_pan_tilt_.tilt += goal->tilt;
    }

    // Indicate that the servos should be moved
    move_head_ = true;
    
    movement_complete_ = false;
}
//---------------------------------------------------------------------------

// This function creates and publishes a joint state message
void HeadControlNode::publishJointState(struct position pan_tilt)
{
    msg_.position[0] = pan_tilt.pan;
    msg_.position[1] = pan_tilt.tilt;
    msg_.header.stamp = ros::Time::now();

    move_head_pub_.publish(msg_); 
}

// Constructor 
HeadControlNode::HeadControlNode(ros::NodeHandle n, std::string name) : as_(n, name, false)
{	
    nh_ = n;

	as_.registerGoalCallback(boost::bind(&HeadControlNode::pointHeadCallback, this));
	as_.start();         
		    	
    // Topic to move head
    move_head_pub_ = nh_.advertise<sensor_msgs::JointState>("pan_tilt_node/joints", 10, true); 
	
    // Obtain any configuration values from the parameter server. If they don't exist use the defaults

    // Joint names
    nh_.param<std::string>("/servo/index0/pan/joint_name", pan_joint_name_, "reserved_pan0");
    nh_.param<std::string>("/servo/index0/tilt/joint_name", tilt_joint_name_, "reserved_tilt0");

    // Maximum angle we can move in one go
    nh_.param("/head/max_step/pan", pan_step_, 0.174533);
    nh_.param("/head/max_step/tilt", tilt_step_, 0.174533);
        
    double pan;      // Pan default position to return to
    double tilt;     // Tilt default position to return to
    nh_.param("/head/position/pan", pan, 0.0);    
    nh_.param("/head/position/tilt", tilt, 0.0);
    default_position_.pan = pan;
    default_position_.tilt = tilt;
	 
    // Set up the the message we will publish
    msg_.name.push_back(pan_joint_name_);
    msg_.name.push_back(tilt_joint_name_);
    msg_.position.push_back(0.0);
    msg_.position.push_back(0.0);

    // We will often return to this position when a task is completed	
    current_pan_tilt_ = default_position_;
    // We don't know where the servo starts from so just jump to the required position    
    // Publish a start position to get the head in a known position.
    publishJointState(current_pan_tilt_);
    
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
    HeadControlNode head_control(n, node_name);	
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

