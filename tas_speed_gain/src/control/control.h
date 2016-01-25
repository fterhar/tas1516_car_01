
/*
 * Created by:  Fynn Terhar
 * Modified by: -
 * Descriptiopn: This class implements the control logic for the speed_gain.
 */

#ifndef CONTROL_H
#define CONTROL_H

#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16MultiArray.h"
#include <math.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include "../rect/rect.h"
#include "../rect/point.h"
#include <tf/transform_listener.h>
#include <cstdarg>
#include <vector>

//Switch to turn boost mode off totally
#define BOOST_MODE_OFF 0 //1

class control
{
public:
    control();

    ros::NodeHandle nh_;
    ros::Publisher  cmd_pub;
    ros::Publisher  speed_gain_pub;
    ros::Publisher  rect_pub;
    ros::Subscriber cmd_sub;
    ros::Subscriber pose_sub; /* To check if inside boost areas */
    ros::Subscriber wii_communication_sub; 
   
    tf::TransformListener listener;
 
    bool  isInBoostArea;
    float speedGainFactor;
 
    /* Here are the Rectangles that define speed boost areas */ 
    Rect R1;
    Rect R2;
    Rect R3;
    Rect R4;

    std_msgs::Int16 control_Mode; /* manual or autonomous */
    std_msgs::Float32 speed_gain_msg;
    geometry_msgs::Twist cmd_msg;
    geometry_msgs::PoseArray rect_msg;

private:
    /* subscribe the cmd message from move_base */
    void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    geometry_msgs::PoseArray rectsToPoseArray(Rect, Rect, Rect, Rect);
    void wiiCommunicationCallback\
	(const std_msgs::Int16MultiArray::ConstPtr& msg);

};

#endif // CONTROL_H
