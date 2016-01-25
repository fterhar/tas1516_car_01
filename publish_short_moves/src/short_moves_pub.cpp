//AC
#include "control/control.h"			//control.cpp contains subscriber to cmd_vel and odom_vel, it publishes on topic servo
#include "ros/ros.h"
#include "std_msgs/Bool.h"

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "short_moves_pub");

  control move_control;
  
  ros::NodeHandle n;

  ros::Publisher short_moves_pub = n.advertise<std_msgs::Bool>("/interrupt", 100);	//publisher to new topic interrupt

  ros::Rate loop_rate(1000); //every second (1 kHz)

  std_msgs::Bool msg;

  unsigned int count = 0;
  while (ros::ok())	//this returns false if e.g. CTRL-C is hit
  {
    /*if (count == 500)	//first accelerate
    {
        msg.data = true;
        ROS_INFO("stop wii");
        short_moves_pub.publish(msg);   //publish to interrupt topic the value false, which then indicates to wii_comunication to stop it's publishing to servo (due to collision reasons)
        move_control.control_servo.x = 1550;
        move_control.control_servo_pub_.publish(move_control.control_servo);	//broadcast message to anyone who is connected
    }

    else if (count == 600)
    {
        move_control.control_servo.x = 1500;
        move_control.control_servo_pub_.publish(move_control.control_servo);	//broadcast message to anyone who is connected
    }

    else if (count == 700)
    {
        move_control.control_servo.x = 1550;
        move_control.control_servo_pub_.publish(move_control.control_servo);	//broadcast message to anyone who is connected
    }

    else if (count == 1500)	//then break
    {
        move_control.control_servo.x = 1500;
        move_control.control_servo_pub_.publish(move_control.control_servo);	//broadcast message to anyone who is connected
    }

    else if (count == 2000)	//after that move backwards...
    {
        move_control.control_servo.x = 1300;
        move_control.control_servo_pub_.publish(move_control.control_servo);	//broadcast message to anyone who is connected
    }

    else if (count == 2100)
    {
        move_control.control_servo.x = 1500;
        move_control.control_servo_pub_.publish(move_control.control_servo);	//broadcast message to anyone who is connected
    }

    else if (count == 2200)
    {
        move_control.control_servo.x = 1300;
        move_control.control_servo_pub_.publish(move_control.control_servo);	//broadcast message to anyone who is connected
    }
    
    else if (count == 3000)	//and break
    {
        msg.data = false;
        ROS_INFO("free wii");    //make wii_comunication know that it now can send to servo topic again by sending false
        short_moves_pub.publish(msg);
        move_control.control_servo.x = 1500;
        move_control.control_servo_pub_.publish(move_control.control_servo);	//broadcast message to anyone who is connected
    }*/


    if (count == 500)	//first accelerate
    {
        msg.data = true;
        ROS_INFO("stop wii");
        short_moves_pub.publish(msg);   //publish to interrupt topic the value false, which then indicates to wii_comunication to stop it's publishing to servo (due to collision reasons)
        move_control.control_servo.x = 1500;
        move_control.control_servo_pub_.publish(move_control.control_servo);	//broadcast message to anyone who is connected
    }
    else if (count == 600)
    {
        move_control.control_servo.x = 1300;
        move_control.control_servo_pub_.publish(move_control.control_servo);	//broadcast message to anyone who is connected
    }

    else if (count == 700)
    {
        move_control.control_servo.x = 1500;
        move_control.control_servo_pub_.publish(move_control.control_servo);	//broadcast message to anyone who is connected
    }

    else if (count == 800)
    {
        move_control.control_servo.x = 1300;
        move_control.control_servo_pub_.publish(move_control.control_servo);	//broadcast message to anyone who is connected
    }

    else if (count == 3000)	//and break
    {
        msg.data = false;
        ROS_INFO("free wii");    //make wii_comunication know that it now can send to servo topic again by sending false
        short_moves_pub.publish(msg);
        move_control.control_servo.x = 1500;
        move_control.control_servo_pub_.publish(move_control.control_servo);	//broadcast message to anyone who is connected
        break;
    }
    ros::spinOnce();	//provide callbacks

    //prevent an overflow
    if(count == 10000)
    {
        count = 8000;
    }
    count++;

    loop_rate.sleep();
    
  }


  return 0;
}

