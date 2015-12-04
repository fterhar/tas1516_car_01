//AC
#include "control/control.h"			//control.cpp contains subscriber to cmd_vel and odom_vel, it publishes on topic servo

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "short_moves_pub");

  control move_control;

  ros::Rate loop_rate(50); //every 250 ms (4 Hz)

  int count = 0;
  while (ros::ok())	//this returns false if e.g. CTRL-C is hit
  {
    

    if (count >= 0 && count < 50)	//first accelerate
    {
	move_control.control_servo.x = 1550;
	move_control.control_servo_pub_.publish(move_control.control_servo);	//broadcast message to anyone who is connected
    }
    
    else if (count >= 50 && count < 100)	//then break
    {
	move_control.control_servo.x = 1500;
	move_control.control_servo_pub_.publish(move_control.control_servo);	//broadcast message to anyone who is connected
    }

    else if (count >= 100 && count < 150)	//after that move backwards...
    {
	move_control.control_servo.x = 1300;
	move_control.control_servo_pub_.publish(move_control.control_servo);	//broadcast message to anyone who is connected
    }
    
    else if (count >= 150 && count < 200)	//and break
    {
	move_control.control_servo.x = 1500;
	move_control.control_servo_pub_.publish(move_control.control_servo);	//broadcast message to anyone who is connected
    }

    else if (count >= 200 && count < 250)	//move again backwards (two times backwards needed after having driven foreward, otherwise it does not work!)
    {
    move_control.control_servo.x = 1300;
    move_control.control_servo_pub_.publish(move_control.control_servo);	//broadcast message to anyone who is connected
    }

    else if (count >= 250 && count < 300)	//and break
    {
    move_control.control_servo.x = 1500;
    move_control.control_servo_pub_.publish(move_control.control_servo);	//broadcast message to anyone who is connected
    }

    ros::spinOnce();	//provide callbacks

    loop_rate.sleep();
    ++count;
  }


  return 0;
}

