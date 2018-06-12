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

#include <actionlib/server/simple_action_server.h>

typedef actionlib::SimpleActionServer<face_recognition_msgs::scan_for_facesAction> Server; 

class HeadControlNode
{
protected:
    Server as_;
    std::string action_name_;	

public:
    HeadControlNode(ros::NodeHandle n, std::string name);
	~HeadControlNode();

private:
    ros::NodeHandle nh_;
		
    servo_msgs::pan_tilt current_pan_tilt_;
    servo_msgs::pan_tilt default_position_; // Neutral position for pan and tilt
		
    ros::Subscriber individual_scan_finished_sub_;
    ros::Publisher start_individual_scan_pub_;
    ros::Publisher move_head_pub_;
	    	   	
    int pan_min_;               // Smallest servo angle for pan
    int pan_max_;               // Maximum servo angle for pan
    int tilt_min_;              // Smallest servo angle for tilt
    int tilt_max_;              // Maximum servo angle for tilt	
		
    bool increase_pan_;         // When true and scanning pan angle will increase, otherwise decrease 
    int pan_step_;              // Angle to increase/decrease pan position by when scanning 
    int tilt_step_;             // Angle to increase/decrease tilt position by when scanning
		
    int total_indv_scans_;     // The total number of individual scans that make up a complete scan
    int scans_complete_;       // Number of scans conducted in this pass

    // This callback is used to kick off the action
    void scanForFacesCallback();

    // This will be called after we receive the result of each individual scan except for the final scan 
    void publishFeedback(float progress, face_recognition_msgs::face_recognition msg);

    // This callback is used when the face recognition node sends the result back for an individual scan		
    void individualScanFinishedCallback(const face_recognition_msgs::face_recognition& msg);
};

#endif // HEAD_CONTROL_NODE_H_
