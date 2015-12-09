
/*
 * Created by:  Fynn Terhar
 * Modified by: -
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "control/control.h"
#include <sstream>
#include "rect/rect.h" 


int main(int argc, char **argv)
{
  ros::init(argc, argv, "speed_gain");

  control speed_gain_controller;

  ros::NodeHandle n;

  ros::Rate loop_rate(50);
  while(ros::ok())  
  {
      speed_gain_controller.\
	tas_cmd_pub.publish(speed_gain_controller.tas_cmd_msg);

      speed_gain_controller.\
	rect_pub.publish(speed_gain_controller.rect_msg);

      ros::spinOnce();

      loop_rate.sleep();
  }

  return 0;
}
