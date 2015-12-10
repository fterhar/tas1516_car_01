#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "tf2_msgs/TFMessage.h"
#include <tf/transform_broadcaster.h>
//#include <tf/tf.h>


//using namespace tf;

float linear_vel_x = 0;
float linear_vel_y = 0;
float angular_vel = 0;

void realCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
  linear_vel_x = odom->twist.twist.linear.x;
  linear_vel_y = odom->twist.twist.linear.y;
  angular_vel = odom->twist.twist.angular.z;
  ROS_INFO("Current velocity: [%lf,%lf,%lf]", linear_vel_x, linear_vel_y, angular_vel); //store x,y,z position values
}

/*void neededCallback(const  typ cmd_vel )
{
  //cmd_vel speichern
}
*/
int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "control_vel");

  ros::NodeHandle n;

  
  ros::Subscriber real_vel = n.subscribe("tas_odom", 1000, realCallback);
  //ros::Subscriber needed_vel = n.subscribe("cmd_vel", 1000, neededCallback);
  ros::Publisher ctrl_vel_pub = nh.advertise<nav_msgs::Odometry>("tas_cmd_vel", 1000);
  

  ros::spin();

  return 0;
}

