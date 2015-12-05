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
    // R4 = Rect(Point(-100.5, -100.71), Point(200.3, 200.15));

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
    /* Transform the received pose to the map frame: */
    tf::StampedTransform transform;
    std::string map_frame("/map");
    std::string base_frame("/base_link");
  
    bool tf_ok = true;
    try {
	control::listener.lookupTransform(map_frame, base_frame, ros::Time(0), transform);
    } catch(tf::TransformException ex) {
	tf_ok = false;
        ROS_WARN(" %s", ex.what());
    }
    
    if(tf_ok) {

	Point p(transform.getOrigin().getX(), transform.getOrigin().getY() );
	if(R1.isInside(p) || R2.isInside(p) || R3.isInside(p) || R4.isInside(p)){
	    ROS_INFO("[INFO] Is in Boost area. Pos: (%f|%f)",p.x, p.y);
        } else {
	    ROS_INFO("[INFO] Is not in Boost area. Pos: (%f|%f)",p.x, p.y);
	}
    } 
}

