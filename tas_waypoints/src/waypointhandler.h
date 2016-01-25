/*
 * Created by:  Fynn Terhar
 * Modified by: -
 * Description: This class declares the waypointhandler. Its core component is the
 * 		goal thread. It runs and interpretes the user input from wii-mote.
		It recordes and replays waypoints to achieve the learning of the robot.
 */

#ifndef WAYPOINTHANDLER_H
#define WAYPOINTHANDLER_H

#define WII_BUTTON_A            4
#define WII_BUTTON_NUNCHUK_C    1

#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16MultiArray.h"
#include <wiimote/State.h>
#include <math.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <cstdarg>
#include <vector>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <actionlib/client/simple_action_client.h>

/* Type for wiimote buttons */
typedef struct{ 
    bool A;
    bool C;		
}WiiBtnState;

// Goal States: GoalThread is implemented as an FSM
enum goal_state_enum{Init, Record, Replay};
typedef enum goal_state_enum Goal_state;


class Waypointhandler
{
    public:
        WiiBtnState setWiiButtonState();
        Waypointhandler();
        WiiBtnState getWiiButtonState();
        bool store_waypoint();
        ros::NodeHandle nh_;

        ros::Subscriber wii_communication_sub; 
   
	// Objects for getting the current robot position on the map
        tf::TransformListener listener;
        tf::StampedTransform transform;

        //vector of goals, with position and orientation
        std::vector<geometry_msgs::Pose> waypoints;

    private:
	// This function is started from the constructor as a boost thread.
        void runGoalThread();
	
	// old and new button state to detect a rising edge.
        WiiBtnState oldBtnState;
        WiiBtnState btnState;

    	void wiiStateCallback(const wiimote::State::ConstPtr& wiiState);
};

#endif
