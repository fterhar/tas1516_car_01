#ifndef CONTROL_H
#define CONTROL_H

#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int16MultiArray.h"
#include <math.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include "../rect/rect.h"
#include "../rect/point.h"
#include <tf/transform_listener.h>

#define BOOST_MODE_OFF 0 //1

class control
{
public:
    control();

    ros::NodeHandle nh_;
    ros::Publisher cmd_pub;
    ros::Publisher tas_cmd_pub;
    ros::Subscriber cmd_sub;
    ros::Subscriber pose_sub; /* To check if inside boost areas */
    ros::Subscriber wii_communication_sub; 
   
    tf::TransformListener listener;
 
    bool  isInBoostArea;
    float speedGainFactor;
 
     /* Here are the Rectangles that define areas where speed boost can be used */ 
     Rect R1;// (Point(25.1, 8.65), Point(21.3, 17.1));
     Rect R2;// (Point(20.3, 17.5), Point(12.5, 20.4));
     Rect R3;// (Point(10.3, 18.3), Point(11.2, 9.46));
     Rect R4;// (Point(12.5, 3.71), Point(20.3, 9.15));

   

    /* Vielleicht später noch wichtig: */
//    std_msgs::Int16 control_Brake; /* flag for brake */
    std_msgs::Int16 control_Mode; /* manual or autonomous */
    geometry_msgs::Twist cmd_msg;
    geometry_msgs::Twist tas_cmd_msg;
    geometry_msgs::PoseStamped pose_msg;

    double cmd_linearVelocity;
    double cmd_angularVelocity;
    double cmd_steeringAngle;

private:
    /* subscribe the cmd message from move_base */
    void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    void wiiCommunicationCallback\
	(const std_msgs::Int16MultiArray::ConstPtr& msg);

};

#endif // CONTROL_H
