//AC
#include "control/control.h"			//control.cpp contains subscriber to cmd_vel and odom_vel, it publishes on topic servo
#include "ros/ros.h"
#include "std_msgs/Bool.h"

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "short_moves_pub");

  control move_control;
  
  ros::NodeHandle n;

  ros::Publisher short_moves_pub = n.advertise<std_msgs::Bool>("/interrupt", 1);

  ros::Rate loop_rate(1); //every second (1 Hz)

  std_msgs::Bool msg;

  int count = 0;
  while (ros::ok())	//this returns false if e.g. CTRL-C is hit
  {
    if (count == 0)	//first accelerate
    {
	msg.data = true;	
	short_moves_pub.publish(msg);
	ROS_INFO("stop others");	
	//ros::spinOnce();
	move_control.control_servo.x = 1550;
	move_control.control_servo_pub_.publish(move_control.control_servo);	//broadcast message to anyone who is connected
    }

    if (count == 1)	//first accelerate
    {
	move_control.control_servo.x = 1550;
	move_control.control_servo_pub_.publish(move_control.control_servo);	//broadcast message to anyone who is connected
    }
    
    else if (count == 2)	//then break
    {
	move_control.control_servo.x = 1500;
	move_control.control_servo_pub_.publish(move_control.control_servo);	//broadcast message to anyone who is connected
    }

    else if (count == 3)	//after that move backwards...
    {
	move_control.control_servo.x = 1300;
	move_control.control_servo_pub_.publish(move_control.control_servo);	//broadcast message to anyone who is connected
    }
    
    else if (count == 4)	//and break
    {
	move_control.control_servo.x = 1500;
	move_control.control_servo_pub_.publish(move_control.control_servo);	//broadcast message to anyone who is connected
	msg.data = false;
	short_moves_pub.publish(msg);
	ROS_INFO("free others");
    }

    //ros::spinOnce();	//provide callbacks

    if(count == 100)
    {
	count = 10;
    }
    count++;

    loop_rate.sleep();
    
  }


  return 0;
}

