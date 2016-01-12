/*
 * Created by:  Fynn Terhar
 * Modified by: -
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "waypointhandler.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "tas_waypoints");

  Waypointhandler waypoint_handler;

  ros::NodeHandle n;

  ros::Rate loop_rate(50);
  while(ros::ok())  
  {
//      speed_gain_controller.\
//	speed_gain_pub.publish(speed_gain_controller.speed_gain_msg);
//
//      speed_gain_controller.\
//	rect_pub.publish(speed_gain_controller.rect_msg);
//
      ros::spinOnce();

      loop_rate.sleep();
  }

  return 0;
}
