
/*
 * Created by:  Fynn Terhar
 * Modified by: -
 * Description: This node defines rectangles. The rectangles are
 * 		placed at the straight parts of the hallways. They
		multiply the cars speed by a factor.
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
	speed_gain_pub.publish(speed_gain_controller.speed_gain_msg);

      speed_gain_controller.\
	rect_pub.publish(speed_gain_controller.rect_msg);

      ros::spinOnce();

      loop_rate.sleep();
  }

  return 0;
}
