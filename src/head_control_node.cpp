// This node accepts the following actions
//    * Scan for faces. During this action the head/camera is moved to a number of positions where
//      individual scans of the current image are taken. The starting head position for the action
//      will be head pan min and head tilt min. The scan will then work up scanning left and right
//      until the tilt max value would be reached.
#include <ros/ros.h>
#include <head_control/head_control_node.h>
#include <std_msgs/Empty.h>
#include <cmath>    // std::abs

// This callback is used to kick off the action to scan all positions
void HeadControlNode::scanForFacesCallback()
{
    face_recognition_msgs::scan_for_facesGoal::ConstPtr goal;
	
    goal = as_.acceptNewGoal();
    ROS_DEBUG("HeadControlNode: Got action goal");

    // Clear the list of those seen
    seen_list_.clear();
    
    scans_complete_ = 0;

    // Here we set up to move the head to the start position and then to request an 
    // individual scan when servo reaches that position
    
    // Set the target position to pan min and tilt min. 
    target_pan_tilt_.pan = pan_min_;
    target_pan_tilt_.tilt = tilt_min_;
	
    // Set the variable that says which direction the pan is going. Start by incrementing
    increase_pan_ = true;
    
    // State what we want to occur when the servo reaches the target position
    process_when_moved_ = requestScan;
	
    // Indicate that the servos should be moved
    move_head_ = true;
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

    scans_complete_++;
	
    if(as_.isPreemptRequested() || !ros::ok())
    {
        // Here we set up to move the head to the default position and then
        // to request preempted when servo reaches that position
        
        ROS_DEBUG("HeadControlNode: %s Preempted", action_name_.c_str());
        
        as_.setPreempted();
        
        // preempted, move to the neutral position
        target_pan_tilt_ = default_position_;
                    
        // State what we want to occur when the servo reaches the target position
        process_when_moved_ = nothing;
	
        // Indicate that the servos should be moved
        move_head_ = true;
    }
    else
    {
        // Scan of individual image complete
        // Calculate the next position of the head/camera
        if(increase_pan_ == true)
        {           
            if(target_pan_tilt_.pan == pan_max_)
            {
                // Last scan was at the edge, move tilt up and then pan the other way
                increase_pan_ = false;
                                	            
	            target_pan_tilt_.tilt += tilt_view_step_;
			
	            if(target_pan_tilt_.tilt > tilt_max_)
	            {
	                all_areas_scanned = true;                			
	            }
	        }                                
            else
            {            
	            target_pan_tilt_.pan = current_pan_tilt_.pan + pan_view_step_;
		
	            if(target_pan_tilt_.pan > pan_max_)
	            {          
	                // Moved out of range, put back on max                   
	                target_pan_tilt_.pan = pan_max_;
                }
	        }
        }
        else
        {
            if(target_pan_tilt_.pan == pan_min_)        
            {
                // Last scan was at the edge, move tilt up and then pan the other way
                increase_pan_ = true;
                
                target_pan_tilt_.tilt += tilt_view_step_;
			
	            if(target_pan_tilt_.tilt > tilt_max_)
	            {
	                all_areas_scanned = true;                			
	            }
	        }
	        else
	        {	                     
                target_pan_tilt_.pan = current_pan_tilt_.pan - pan_view_step_;
		
	            if(target_pan_tilt_.pan < pan_min_)
	            {
	                // Moved out of range, put back on min
        	        target_pan_tilt_.pan = pan_min_;
        	    }
	        }
        }
		   
	    // Add any faces seen to the stored list
	    if(msg.ids_detected.size() > 0)
        {  
            for(unsigned long x = 0; x < msg.ids_detected.size(); x++)
            {
                FaceSeen face_detected;
                face_detected.id = msg.ids_detected[x];
                face_detected.name = msg.names_detected[x];             

                if(haveWeSeenThisPerson(face_detected) == false)
                {                                    
                    ROS_DEBUG("HeadControlNode: I have seen %s", msg.names_detected[x].c_str());
                }
            }          
        }
	    
        if(all_areas_scanned == true)
        {              		    
		    // Note here in the result we send a list of all those people seen
		    
	        // Send action result, the seen list
	        face_recognition_msgs::scan_for_facesResult result;	        
	          	        
	        // Iterate through the faces seen adding the the result
	        for (std::list<FaceSeen>::const_iterator iterator = seen_list_.begin(), end = seen_list_.end(); iterator != end; ++iterator)
	        {	            	           	            
	            result.detected.ids_detected.push_back(iterator->id);
	            result.detected.names_detected.push_back(iterator->name);	            	       
	        }
		
            as_.setSucceeded(result);
            
            // Move the head/camera back to default position             
	        target_pan_tilt_ = default_position_;
	            
	        // State what we want to occur when the servo reaches the target position
            process_when_moved_ = nothing;
	
            // Indicate that the servos should be moved
            move_head_ = true;
        }
        else
        {        
	        // Calculate percentage complete
        	percentage_complete = ((float)scans_complete_ / (float)total_indv_scans_) * 100.0;
				    
		    // Note that here in the feedback we only send those just seen, even if seen before
		        
        	// Send feedback which includes the result of the last individual scan
	        publishFeedback(percentage_complete, msg); 	        
		
            // target pan/tilt position set above            	        
	            
	        // State what we want to occur when the servo reaches the target position (i.e. next scan)
            process_when_moved_ = requestScan;
	
            // Indicate that the servos should be moved
            move_head_ = true;
        }	
    }
}
//---------------------------------------------------------------------------

// Function used to keep track of who has been seen
bool HeadControlNode::haveWeSeenThisPerson(FaceSeen face_detected)
{
    bool ret_val = true;

    // Is this person already in our list of people seen
    std::list<FaceSeen>::iterator it = std::find_if(seen_list_.begin(), seen_list_.end(),
    boost::bind(&FaceSeen::id, _1) == face_detected.id);

    if(it == seen_list_.end())
    {
        // Not seen before, add to seen list
        seen_list_.insert(it, face_detected);

        ret_val = false;
    }

    return ret_val;
}
//---------------------------------------------------------------------------

// Function to move the servos if required by a step amount. This is to stop the head shuddering if the servo
// is moved to the target position in one movement. The function also carries out any process required when
// the servo reaches the target.
void HeadControlNode::moveServo()
{
    if(move_head_ == true)
    {
        // Still processing servo movement
        if(movement_complete_ == true)
        {
            // We have reached the target but may be giving time to settle
            loop_count_down_--;
            
            if(loop_count_down_ <= 0)
            {
                movement_complete_ = false;
                move_head_ = false;
                
                if(process_when_moved_ == requestScan)
                {
                    // Request the next individual scan for this position
                    std_msgs::Empty mt;
                    start_individual_scan_pub_.publish(mt);
                }
            }
        }
        else
        {
            if((target_pan_tilt_.pan == current_pan_tilt_.pan) && (target_pan_tilt_.tilt == current_pan_tilt_.tilt))
            {
                // Last time around we must have requested the final move
                movement_complete_ = true;
                if(process_when_moved_ == requestScan)
                {
                    // Allow time for head to steady before requesting scan of image
                    loop_count_down_ = 5;
                }
                else
                {
                    loop_count_down_ = 1;
                }
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
    pan_step_ = 10;     // Maximum step movment for pan servo 
    tilt_step_ = 10;    // Maximum step movment for tilt servo 
    pan_view_step_ = 10;     // Angle to increase/decrease pan position by when scanning 
    tilt_view_step_ = 10;    // Angle to increase tilt position by when scanning 
	
    // Obtain any configuration values from the parameter server. If they don't exist use the defaults above
    nh_.param("/servo/index0/pan_min", pan_min_, pan_min_);
    nh_.param("/servo/index0/pan_max", pan_max_, pan_max_);
    nh_.param("/servo/index0/tilt_min", tilt_min_, tilt_min_);
    nh_.param("/servo/index0/tilt_max", tilt_max_, tilt_max_);
    nh_.param("/head/view_step/pan", pan_view_step_, pan_view_step_);
    nh_.param("/head/view_step/tilt", tilt_view_step_, tilt_view_step_);
    nh_.param("/head/max_step/pan", pan_step_, pan_step_);
    nh_.param("/head/max_step/tilt", tilt_step_, tilt_step_);
        
    int pan = 90;      // Pan position to return to once a task is complete
    int tilt = 45;     // Tilt position to return to once a task is complete
    nh_.param("/head/position/pan", pan, pan);    
    nh_.param("/head/position/tilt", tilt, tilt);
    default_position_.pan = (int)pan;
    default_position_.tilt = (int)tilt;
    
    // Calculate the total number of individual scans for % complete
    total_indv_scans_ = (((pan_max_ - pan_min_) / pan_view_step_) + 1) * (((tilt_max_ - tilt_min_) / tilt_view_step_) + 1) - 1;
	
    // This start position may be so the user can access the on screen keyboard. 
    // We will often return to this position when a task is completed	
    current_pan_tilt_ = default_position_;
    // We don't know where the servo starts from so just jump to the required position    
    // Publish a start position to get the head in a known position.
    move_head_pub_.publish(current_pan_tilt_);
    
    move_head_ = false;
    movement_complete_ = false;
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

