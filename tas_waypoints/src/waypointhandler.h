/*
 * Created by:  Fynn Terhar
 * Modified by: -
 */

#ifndef WAYPOINTHANDLER_H
#define WAYPOINTHANDLER_H

#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16MultiArray.h"
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
        ros::Publisher  rect_pub;
        ros::Subscriber wii_communication_sub; 
   
        tf::TransformListener listener;
        tf::StampedTransform transform;

        //vector of goals, with position and orientation
        std::vector<geometry_msgs::Pose> waypoints;
        std_msgs::Float32 speed_gain_msg;

//	void doneCb(const actionlib::SimpleClientGoalState& scgs,\
//		    const move_base_msgs::MoveBaseResultConstPtr& res); 
//        void activeCb();
//	void feedbackCb(\
//		  const move_base_msgs::MoveBaseFeedbackConstPtr& fb); 

    private:
        void runGoalThread();

        WiiBtnState oldBtnState;
        WiiBtnState btnState;


        void wiiCommunicationCallback(\
		  const std_msgs::Int16MultiArray::ConstPtr& msg);

};

#endif
