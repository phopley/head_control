#ifndef HEAD_CONTROL_NODE_H_
#define HEAD_CONTROL_NODE_H_
// Header file for the head control node.
// It accepts the following actions
//    * Scan for faces. During this action the head/camera is moved to a number of positions where
//      individual scans of the current image are taken.

#include <ros/ros.h>
#include <servo_msgs/pan_tilt.h>
#include <face_recognition_msgs/face_recognition.h>
#include <face_recognition_msgs/scan_for_facesAction.h>
#include <face_recognition_msgs/scan_for_facesActionFeedback.h>
#include <face_recognition_msgs/scan_for_facesActionResult.h>
#include <std_msgs/String.h>

#include <actionlib/server/simple_action_server.h>

typedef actionlib::SimpleActionServer<face_recognition_msgs::scan_for_facesAction> Server; 

struct FaceSeen
{
    unsigned int id;
    std::string name;
};

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
		
    ros::Subscriber individual_scan_finished_sub_;
    ros::Subscriber manual_sub_;
    ros::Publisher start_individual_scan_pub_;
    ros::Publisher move_head_pub_;
    
    std::list<FaceSeen> seen_list_; // List of faces seen recently
	    	   	
    int pan_min_;               // Smallest servo angle for pan
    int pan_max_;               // Maximum servo angle for pan
    int tilt_min_;              // Smallest servo angle for tilt
    int tilt_max_;              // Maximum servo angle for tilt	
		
    bool increase_pan_;         // When true and scanning pan angle will increase, otherwise decrease 
    int pan_step_;              // Maximum angle we can move the pan servo in one go 
    int tilt_step_;             // Maximum angle we can move the tilt servo in one go
    int pan_view_step_;         // Angle to increase/decrease pan position between scans
    int tilt_view_step_;        // Angle to increase/decrease tilt position between scans
		
    int total_indv_scans_;      // The total number of individual scans that make up a complete scan
    int scans_complete_;        // Number of scans conducted in this pass
    
    int loop_count_down_;       // Used to give time for head to settle before grabing image from camera
    
    bool move_head_;            // Set to true when servos should move to target position
    bool movement_complete_;    // Set to true when servo movement complete and next process requested
    
    enum MovementComplete {requestScan, nothing};
    MovementComplete process_when_moved_;    // Process to do when servo reaches target position

    // This callback is used to kick off the action
    void scanForFacesCallback();

    // This will be called after we receive the result of each individual scan except for the final scan 
    void publishFeedback(float progress, face_recognition_msgs::face_recognition msg);

    // This callback is used when the face recognition node sends the result back for an individual scan		
    void individualScanFinishedCallback(const face_recognition_msgs::face_recognition& msg);
    
    // This calback is used when a command to manually move the head/camera is received
    void manualMovementCallback(const std_msgs::String& msg);
    
    // Function used to keep track of who has been seen
    bool haveWeSeenThisPerson(FaceSeen face_detected);   
};

#endif // HEAD_CONTROL_NODE_H_
