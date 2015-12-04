#include "control.h"

control::control()
{
    cmd_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    cmd_sub = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 1000, &control::cmdCallback,this);

    pose_sub = nh_.subscribe<geometry_msgs::PoseStamped>("slam_out_pose", 1000, &control::poseCallback,this);

  /* Braucht man vielleicht später: */
  //  wii_communication_sub = nh_.subscribe<std_msgs::Int16MultiArray>("wii_communication",1000,&control::wiiCommunicationCallback,this);
    
     R1 = Rect(Point(25.1, 8.65), Point(21.3, 17.1));
     R2 = Rect(Point(20.3, 17.5), Point(12.5, 20.4));
     R3 = Rect(Point(10.3, 18.3), Point(11.2, 9.46));
     R4 = Rect(Point(12.5, 3.71), Point(20.3, 9.15));

}

//Subscribe to the local planner and map the steering angle (and the velocity-but we dont do that here-) to pulse width modulation values.
void control::cmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
   // if () 
    if (msg != 0){
	cmd_msg = *msg;
    }
}

void control::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
   // if (pose)
	//pose_msg = *pose;
    
}

// a flag method that tells us if we are controlling the car manually or automatically
//void control::wiiCommunicationCallback(const std_msgs::Int16MultiArray::ConstPtr& msg)
//{
//    control_Mode.data = msg->data[0];
//    control_Brake.data = msg->data[1];
//}

//geometry_msgs::Vector3 control::P_Controller()
//{
//    current_ServoMsg.x = previous_ServoMsg.x + Fp*(cmd_linearVelocity - odom_linearVelocity);

//    current_ServoMsg.y = cmd_steeringAngle;


//    if(current_ServoMsg.x > 1580)
//    {
//        current_ServoMsg.x = 1580;
//    }
//    else if(current_ServoMsg.x < 1300)
//    {
//        current_ServoMsg.x = 1300;
//    }

//    if(current_ServoMsg.y > 2000)
//    {
//        current_ServoMsg.y = 2000;
//    }
//    else if(current_ServoMsg.y < 1000)
//    {
//        current_ServoMsg.y = 1000;
//    }

//    previous_ServoMsg = current_ServoMsg;

//    return current_ServoMsg;
//}
