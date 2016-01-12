 /**
 * This node sends fixed goals to move base via ROS Action API and receives feedback via callback functions.
 */


#include <iostream>
#include "waypointhandler.h"

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>\
	 MoveBaseClient;

/**
 * Callback function
 */
void doneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result) {
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
}

/**
 * Callback function, called once when the goal becomes active
 */
void activeCb() {
    ROS_INFO("Goal just went active");
}

/**
 * Callback function, called every time feedback received for the goal
 */
void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) {
    ROS_INFO("[X]:%f [Y]:%f [W]: %f [Z]: %f", feedback->base_position.pose.position.x,feedback->base_position.pose.position.y,feedback->base_position.pose.orientation.w, feedback->base_position.pose.orientation.z);
}
Waypointhandler::Waypointhandler(){
 
    /* Init Buttonstates as false */    
    btnState.A = false;
    btnState.C = true;

    /* Init the goal_giver instance */
//    rect_pub = \
//	nh_.advertise<geometry_msgs::PoseArray>\
//		("boost_areas", 100, false);
//
//    speed_gain_pub = \
//	nh_.advertise<std_msgs::Float32>("speed_gain_msg", 1);
//
//    cmd_sub = \
//	nh_.subscribe<geometry_msgs::Twist>\
//	("cmd_vel", 1000, &control::cmdCallback,this);
//
//    pose_sub = \
//	nh_.subscribe<geometry_msgs::PoseStamped>\
//	("slam_out_pose", 1000, &control::poseCallback,this);
//
    wii_communication_sub = nh_.subscribe<std_msgs::Int16MultiArray>\
	("wii_communication",1000,&Waypointhandler::wiiCommunicationCallback,\
	 this);
 
}
 /**
 * Record the current position as a waypoint and store the waypoint
 * in the respective array
 */
bool Waypointhandler::store_waypoint(){
    ROS_INFO("Recording a waypoint...");

    /* Transform the received pose to the map frame: */
    std::string map_frame("/map");
    std::string base_frame("/base_link");

    bool tf_ok = true;
    try {
        Waypointhandler::listener.lookupTransform\
			(map_frame, base_frame, ros::Time(0), \
		 	 Waypointhandler::transform);
    } catch(tf::TransformException ex) {
	tf_ok = false;
        ROS_WARN(" %s", ex.what());
    }

    if(tf_ok) {

	int px = transform.getOrigin().getX();
	int py = transform.getOrigin().getY();
	int pz = transform.getOrigin().getZ();
	int rx = transform.getRotation().getAxis().x();
	int ry = transform.getRotation().getAxis().y();
	int rz = transform.getRotation().getAxis().z();
	int rw = transform.getRotation().getAxis().w();

	geometry_msgs::Pose waypoint;
	waypoint.position.x    = px;
	waypoint.position.y    = py;
	waypoint.position.z    = pz;
	waypoint.orientation.x = rx;
	waypoint.orientation.y = ry;
	waypoint.orientation.z = rz;
	waypoint.orientation.w = rw;

	Waypointhandler::waypoints.push_back( waypoint );
    } else {
	ROS_WARN("Position of Robot was not received properly!");
    }
    
    return tf_ok;
}  

void Waypointhandler::wiiCommunicationCallback(\
		const std_msgs::Int16MultiArray::ConstPtr& msg){
    btnState.C = msg->data[0];
    btnState.A = msg->data[3];//TODO: CHECK INDEX!
	
}



/**
 * This runs as a thread and collects all data and also replays them 
 * later on. 
 */
void Waypointhandler::runGoalThread(){

    int i = 0;
    Goal_state state = Init;
    bool replay_initialised = false;
    MoveBaseClient ac("move_base", true); // action client to spin a thread by default

    while (!ac.waitForServer(ros::Duration(5.0))) { // wait for the action server to come up
	ROS_INFO("Waiting for the move_base action server to come up");
    }
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map"; // set target pose frame of coordinates

    while(ros::ok()){

	switch(state){
 	    case Init:   
	        if(btnState.C == 0){
		    state = Record;
	        } else {
		    state = Replay;
	        }
	        break;

	    case Record:
	        // Record the waypoints here
	        if( (btnState.A == true) && (oldBtnState.A == false) ){
	            if( store_waypoint() ){
		        ROS_INFO("... Waypoint successfully stored!");
	            }
	        }
		if(btnState.C == true){
		    state = Replay;
		} else {
		    state = Record;
		}
	        oldBtnState = btnState;

	    case Replay:
		// Replay the recorded waypoints here
		/* loop over all goal points, point by point */
	    	for(; i < waypoints.size(); ++i) { 

                    goal.target_pose.header.stamp = ros::Time::now(); // set current time
	            goal.target_pose.pose = waypoints.at(i);

	            ROS_INFO("Sending goal");

	            ac.sendGoal(goal, 
		        	&doneCb, 
		        	&activeCb, 
		        	&feedbackCb
		    ); // send goal and register callback handler

	            ac.waitForResult(); // wait for goal result

                    if( ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED ) {
	            	ROS_INFO("The base moved to %d goal", i);
	            } else {
	            	ROS_INFO("The base failed to move to %d goal for some reason", i);
	            }//end of if

		    /* Check if C-Button is no longer pressed and go back to record mode */
		    if( btnState.C == false ){
			state = Record;
			break;
		    }

	    	}//end of for	
            }//end of case 
    }//end of while(ros::ok)        
}
