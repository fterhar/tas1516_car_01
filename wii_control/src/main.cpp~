//AC
#include "wii_lib/wii_lib.h"
#include "std_msgs/Bool.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "teleop_wii");
    wii_lib teleop_wii;
    
    ros::NodeHandle n;


    ros::Rate loop_rate(50);

    static int count = 0;

    while(ros::ok())
    {

        teleop_wii.wii_communication_pub.publish(teleop_wii.wii_state_);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}


