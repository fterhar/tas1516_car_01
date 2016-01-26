#include "control/control.h"
#include <std_msgs/Bool.h>

bool stop = false;

void stopCallback(const std_msgs::Bool::ConstPtr& stp)
{
     ROS_INFO("flag changed");
     stop = stp.get()->data;	//received flag by short_moves_pub node if it publishes to servo
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "autonomous_control");
    ros::NodeHandle nh_;
    ros::Subscriber interrupt = nh_.subscribe<std_msgs::Bool>("/interrupt",100, &stopCallback);
    control autonomous_control;

    ros::Rate loop_rate(50);



    while(ros::ok())
    {
        if(autonomous_control.control_Mode.data==0)
        {
            ROS_INFO("Manually Control!");
        }
        else
        {
            if(autonomous_control.control_Brake.data==1)
            {
                autonomous_control.control_servo.x=1500;
                autonomous_control.control_servo.y=1500;
            }
            else
            {
                ROS_INFO("Automatic Control!");
                if(autonomous_control.cmd_linearVelocity>0)
                {
                    autonomous_control.control_servo.x = 1500 + 55 * autonomous_control.speed_gain_factor; //1550;
                  //autonomous_control.control_servo.x = 1500 + 400 * autonomous_control.cmd_linearVelocity;    //AC velocity adaption positive
                  //autonomous_control.control_servo.x = 1550;
                }
                else if(autonomous_control.cmd_linearVelocity<0)
                {
                  //autonomous_control.control_servo.x = 1500 + 200 * autonomous_control.cmd_linearVelocity;    //AC velocity adaption negative
                    autonomous_control.control_servo.x = 1300;
                }
                else
                {
                    autonomous_control.control_servo.x = 1500;
                }

                autonomous_control.control_servo.y = autonomous_control.cmd_steeringAngle + 80;
            }

            if(!stop)
            {
                ROS_INFO("active");
                autonomous_control.control_servo_pub_.publish(autonomous_control.control_servo);
            }
            else
                ROS_INFO("blocked");



        }

        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;

}
