//AC
#include "ros/ros.h"
#include "std_msgs/Int16MultiArray.h"	//include message type

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "keyboard_sim_node");

  ros::NodeHandle n;

  ros::Publisher pub = n.advertise<std_msgs::Int16MultiArray>("/wii_communication", 1000, true);	

  ros::Rate loop_rate(100); //every 10 ms (100 Hz)

  

  while (ros::ok())	//this returns false if e.g. CTRL-C is hit
  {
    std_msgs::Int16MultiArray msg;

    
    
    pub.publish(msg);	//broadcast message to anyone who is connected

    ros::spinOnce();	//provide callbacks

    loop_rate.sleep();
  }


  return 0;
}

