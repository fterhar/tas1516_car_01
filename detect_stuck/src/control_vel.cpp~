//AC
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "tf2_msgs/TFMessage.h"
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include "control/control.h"
#include <boost/circular_buffer.hpp>
 
float real_linear_vel_x = 0;
float real_linear_vel_y = 0;
float real_angular_vel_z = 0;

float needed_linear_vel_x;
float needed_linear_vel_y;
float needed_angular_vel_z;

const int length = 5;

boost::circular_buffer<float> xBuffer(length);
boost::circular_buffer<float> yBuffer(length);

/* Calculate mean of last values */
float mean(boost::circular_buffer<float>* vals){\
    float mean = 0;

    for(int i = 0 ;i < vals->size(); i++){
        mean += vals->at(i);

    }
    mean /= vals->size();

    return mean;
}



static bool flg = false;

//receive real velocity values based on fake odometry and set the flag to true
void realCallback(const nav_msgs::Odometry::ConstPtr& odom)
{

  real_linear_vel_x = odom->twist.twist.linear.x;
  xBuffer.push_back(odom->twist.twist.linear.x);

  real_linear_vel_y = odom->twist.twist.linear.y;
  yBuffer.push_back(odom->twist.twist.linear.y);

  real_angular_vel_z = odom->twist.twist.angular.z;
  flg = true;
}

//receive the requested velocity values
void neededCallback(const geometry_msgs::Twist::ConstPtr& twst)
{
  needed_linear_vel_x = twst->linear.x;
  needed_linear_vel_y = twst->linear.y;
  needed_angular_vel_z = twst->angular.z;
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "control_vel");

  control move_control;

  ros::NodeHandle n;

  ros::Subscriber real_vel = n.subscribe("odom", 1000, realCallback);
  ros::Subscriber needed_vel = n.subscribe("cmd_vel", 1000, neededCallback);
  
  ros::Rate r(1000);

  int cnt = 0;

  while(n.ok())
  {
     //make bang-bang-control based on the arrival of the real velocity value (then the flag will be true)
     ros::spinOnce();
     if(flg)
     {
	ROS_INFO("Both received!");



    if((fabs(needed_angular_vel_z) > 0 && fabs(mean(&xBuffer)) < 0.01 && fabs(mean(&yBuffer)) < 0.01))
	{
		cnt++;
        ROS_INFO("count %i", cnt);
        ROS_INFO("Z: %f, xMean: %f, yMean:%f", (fabs(needed_angular_vel_z)), (fabs(mean(&xBuffer))), (fabs(mean(&yBuffer))));
        ROS_INFO("Needed velocity: [%lf,%lf,%lf]", needed_linear_vel_x, needed_linear_vel_y, needed_angular_vel_z);
        ROS_INFO("Current velocity: [%lf,%lf,%lf]", real_linear_vel_x, real_linear_vel_y, real_angular_vel_z);

    } else {
        cnt = 0;
    }
	
    if(cnt > 5)
	{
        ROS_INFO("other node started");
		system("rosrun publish_short_moves short_moves_pub");
		cnt = 0;
	}

	/*	
	
	//when threshold value is reached increase/decrease (depending on reaching upper or lower threshold) the velocity by publishing 	//a appropriate servo value	
	if(needed_linear_vel_x > 0.1 || needed_linear_vel_y > 0.1)
	{
		if(needed_linear_vel_x - real_linear_vel_x > 0.1 || needed_linear_vel_y - real_linear_vel_y > 0.1)
		{
		   move_control.control_servo.x = 1550;
		   move_control.control_servo_pub_.publish(move_control.control_servo);	//broadcast message to anyone who is connected
		   ROS_INFO("too slow - forwards");
		}
		if(needed_linear_vel_x - real_linear_vel_x < -0.1 || needed_linear_vel_x - real_linear_vel_x < -0.1)
		{
		   move_control.control_servo.x = 1500;
		   move_control.control_servo_pub_.publish(move_control.control_servo);	//broadcast message to anyone who is connected
		   ROS_INFO("too fast - forwards");
		}
	}
	if(needed_linear_vel_x < -0.1 || needed_linear_vel_y < -0.1)
	{
		if(needed_linear_vel_x - real_linear_vel_x < -0.1 || needed_linear_vel_y - real_linear_vel_y < -0.1)
		{
		   move_control.control_servo.x = 1300;
		   move_control.control_servo_pub_.publish(move_control.control_servo);	//broadcast message to anyone who is connected
		   ROS_INFO("too slow - backwards");
		}
		if(needed_linear_vel_x - real_linear_vel_x > 0.1 || needed_linear_vel_y - real_linear_vel_y > 0.1)
		{
		   move_control.control_servo.x = 1500;
		   move_control.control_servo_pub_.publish(move_control.control_servo);	//broadcast message to anyone who is connected
		   ROS_INFO("too fast - backwards");
		}
	}*/
	flg = false;
     }
     r.sleep();
  }

  return 0;
}

