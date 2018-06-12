// It node accepts the following actions
//    * Scan for faces. During this action the head/camera is moved to a number of positions where
//      individual scans of the current image are taken. The starting head position for the action
//      will be head pan min and head tilt min. The scan will then work up scanning left and right
//      until the tilt max value would be reached.
#include <ros/ros.h>
#include <head_control/head_control_node.h>
#include <std_msgs/Empty.h>

// This callback is used to kick off the action to scan all positions
void HeadControlNode::scanForFacesCallback()
{
    std_msgs::Empty mt;
    face_recognition_msgs::scan_for_facesGoal::ConstPtr goal;
	
    goal = as_.acceptNewGoal();
    ROS_DEBUG("HeadControlNode: Got action goal");

    scans_complete_ = 0;

    // Here we send a message to move the head and then to grab a frame
    
    // Move the current position to pan min and tilt min. 
    // When we get back here we have completed the scan
    current_pan_tilt_.pan = pan_min_;
    current_pan_tilt_.tilt = tilt_min_;
    move_head_pub_.publish(current_pan_tilt_);
	
    // Set the variable that says which direction the pan is going. Start by incrementing
    increase_pan_ = true;
	
    // Publish an empty message on the topic to request an individual image scan
    start_individual_scan_pub_.publish(mt);
}
//---------------------------------------------------------------------------

// This will be called after we receive the result of each individual scan except for the final scan 
void HeadControlNode::publishFeedback(float progress, face_recognition_msgs::face_recognition msg)
{
    face_recognition_msgs::scan_for_facesFeedback feedback;
    feedback.progress = progress;
    feedback.detected = msg;    
    as_.publishFeedback(feedback);
}
//---------------------------------------------------------------------------

// This callback is used when the face recognition node sends the result back for an individual scan		
void HeadControlNode::individualScanFinishedCallback(const face_recognition_msgs::face_recognition& msg)
{
    bool all_areas_scanned = false;
    float percentage_complete;
    std_msgs::Empty mt;

    scans_complete_++;
	
    if(as_.isPreemptRequested() || !ros::ok())
    {
        // preempted, move to the neutral position
        current_pan_tilt_ = default_position_;
        
        ROS_DEBUG("HeadControlNode: %s Preempted", action_name_.c_str());    

        as_.setPreempted();
        
        // Move the head/camera
        move_head_pub_.publish(current_pan_tilt_);
    }
    else
    {
        // Scan of individual image complete
        // Calculate the next position of the head/camera
        if(increase_pan_ == true)
        {
	        current_pan_tilt_.pan += pan_step_;
		
	        if(current_pan_tilt_.pan > pan_max_)
	        {          
	            // Moved out of range
                increase_pan_ = false;
	            current_pan_tilt_.pan -= pan_step_;
			
	            // Move tilt up
	            current_pan_tilt_.tilt += tilt_step_;
			
	            if(current_pan_tilt_.tilt > tilt_max_)
	            {
	                all_areas_scanned = true;                			
	            }
	        }
        }
        else
        {        
            current_pan_tilt_.pan -= pan_step_;
		
	        if(current_pan_tilt_.pan < pan_min_)
	        {
	            // Moved out of range
                increase_pan_ = true;

        	    current_pan_tilt_.pan += pan_step_;
			
	            // Move tilt up
	            current_pan_tilt_.tilt += tilt_step_;	
            
	            if(current_pan_tilt_.tilt > tilt_max_)
	            {
	                all_areas_scanned = true;                			
	            }
	        }
        }
	
        if(all_areas_scanned == true)
        {
	        // set position to default position
	        current_pan_tilt_ = default_position_;	               
		
	        // Send action result based on msg
	        face_recognition_msgs::scan_for_facesResult result;
	        result.detected = msg;	        
		
            as_.setSucceeded(result);
            
            // Move the head/camera
            move_head_pub_.publish(current_pan_tilt_);
        }
        else
        {        
	        // Calculate percentage complete
        	percentage_complete = ((float)scans_complete_ / (float)total_indv_scans_) * 100.0;
		
        	// Send feedback which includes the result of the last individual scan
	        publishFeedback(percentage_complete, msg); 	        
		
	        // Send message requesting next individual image scan			
	        start_individual_scan_pub_.publish(mt);
	        
	        // Move the head/camera
            move_head_pub_.publish(current_pan_tilt_);
        }	
    }
}
//---------------------------------------------------------------------------

// Constructor 
HeadControlNode::HeadControlNode(ros::NodeHandle n, std::string name) : as_(n, name, false), action_name_(name)
{	
    nh_ = n;
	
    as_.registerGoalCallback(boost::bind(&HeadControlNode::scanForFacesCallback, this));
    as_.start();
  
    individual_scan_finished_sub_ =
        nh_.subscribe("/face_recognition_node/result", 1, &HeadControlNode::individualScanFinishedCallback, this);          
		
    // Topic to instruct the scan of the current image
    start_individual_scan_pub_ = nh_.advertise<std_msgs::Empty>("face_recognition_node/start", 1);
	
    // Topic to move head
    move_head_pub_ = nh_.advertise<servo_msgs::pan_tilt>("pan_tilt_node/head_position", 10);	
	
    pan_min_ = 0;       // Smallest servo angle for pan
    pan_max_ = 180;     // Maximum servo angle for pan
    tilt_min_ = 0;      // Smallest servo angle for tilt
    tilt_max_ = 180;    // Maximum servo angle for tilt  
    pan_step_ = 10;     // Angle to increase/decrease pan position by when scanning 
    tilt_step_ = 10;    // Angle to increase tilt position by when scanning 
	
    // Obtain any configuration values from the parameter server. If they don't exist use the defaults above
    nh_.param("/servo/index0/pan_min", pan_min_, pan_min_);
    nh_.param("/servo/index0/pan_max", pan_max_, pan_max_);
    nh_.param("/servo/index0/tilt_min", tilt_min_, tilt_min_);
    nh_.param("/servo/index0/tilt_max", tilt_max_, tilt_max_);
    nh_.param("/head/step/pan", pan_step_, pan_step_);
    nh_.param("/head/step/tilt", tilt_step_, tilt_step_);
        
    int pan = 90;      // Pan position to return to once a task is complete
    int tilt = 45;     // Tilt position to return to once a task is complete
    nh_.param("/head/position/pan", pan, pan);    
    nh_.param("/head/position/tilt", tilt, tilt);
    default_position_.pan = (int)pan;
    default_position_.tilt = (int)tilt;
    
    // Calculate the total number of individual scans for % complete
    total_indv_scans_ = (((pan_max_ - pan_min_) / pan_step_) + 1) * (((tilt_max_ - tilt_min_) / tilt_step_) + 1) - 1;
	
    // This start position may be so the user can access the on screen keyboard. 
    // We will often return to this position when a task is completed	
    current_pan_tilt_ = default_position_;    
    // Publish a start position to get the head in a known position.
    move_head_pub_.publish(current_pan_tilt_);
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
    ros::spin();
    return 0;
}

