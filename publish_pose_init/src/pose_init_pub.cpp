//AC
#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"	//include message type

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "pose_init_pub");

  ros::NodeHandle n;

  ros::Publisher pose_init_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, true);	//publish a message of type PoseWithCovarianceStamped. This data is published on topic /initialpose with buffer of 1. Send only if subscriber is there (true)

  ros::Rate loop_rate(100); //every 10 ms (100 Hz)

  int count = 0;
  while (ros::ok())	//this returns false if e.g. CTRL-C is hit
  {
    geometry_msgs::PoseWithCovarianceStamped msg;

    msg.header.frame_id = "map";	//use map as reference
    msg.pose.pose.position.x = 11.388551712;		//x-position of initialpose
    msg.pose.pose.position.y = 19.2560691833;		//y-position of initialpose
    msg.pose.pose.position.z = 0;		//z-position of initialpose
    msg.pose.covariance =  {0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942};	//covariance data of initialpose
    msg.pose.pose.orientation.z = -0.721992788162;		//z-orientation of initialpose
    msg.pose.pose.orientation.w = 0.691900580895;		//rotation of initialpose

    if (count == 0)	//send only once
    {
	pose_init_pub.publish(msg);	//broadcast message to anyone who is connected
    }

    ros::spinOnce();	//provide callbacks

    loop_rate.sleep();
    ++count;
  }


  return 0;
}

