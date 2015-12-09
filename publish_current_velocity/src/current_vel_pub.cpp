#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "tf2_msgs/TFMessage.h"
#include <tf/transform_broadcaster.h>
//#include <tf/tf.h>


//using namespace tf;


void odomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
ROS_INFO("I received odom: [%f,%f,%f]", odom->twist.twist.linear.x, odom->pose.pose.position.y,odom->pose.pose.position.z); //store x,y,z position values
//ROS_INFO("velocity: %f", odom->twist.twist.linear.x, odom->pose.pose.position.y,odom->pose.pose.position.z);
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "current_vel_pub");

  

  ros::NodeHandle n;

  
  ros::Subscriber sub = n.subscribe("odom", 1000, odomCallback);

  
  ros::spin();

  return 0;
}

/*
void odomCallback(const tf2_msgs::TFMessage::ConstPtr& odom)
{
	geometry_msgs::Twist test;
	//tf::Transformer tef;
	//tef.lookupTwist("bar", "odom", ros::Time(0.5), ros::Duration(0.1), test);

//ROS_INFO("I received..: %f", Transformer::lookupTwist()->twist.twist.linear.x); //store x,y,z position values
//ROS_INFO("velocity: %f", odom->twist.twist.linear.x, odom->pose.pose.position.y,odom->pose.pose.position.z);
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "current_vel_pub");

  

  ros::NodeHandle n;

  
  ros::Subscriber current_vel_pub = n.subscribe("tf", 1000, &odomCallback);

  
  ros::spin();

  return 0;
}*/
