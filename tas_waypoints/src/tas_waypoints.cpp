/*
 * Created by:  Fynn Terhar
 * Modified by: -
 * Description: This node lets the user record waypoints on the fly.
 * They serve as training input so that the robot can learn a path.
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
      ros::spinOnce();

      loop_rate.sleep();
  }

  return 0;
}
