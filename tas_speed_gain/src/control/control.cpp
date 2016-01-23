/*
 * Created by:  Fynn Terhar
 * Modified by: -
 */

#include "control.h"

control::control()
{
    rect_pub = \
	nh_.advertise<geometry_msgs::PoseArray>\
		("boost_areas", 100, false);

    speed_gain_pub = \
	nh_.advertise<std_msgs::Float32>("speed_gain_msg", 1);

    cmd_sub = \
	nh_.subscribe<geometry_msgs::Twist>\
	("cmd_vel", 1000, &control::cmdCallback,this);

    pose_sub = \
	nh_.subscribe<geometry_msgs::PoseStamped>\
	("slam_out_pose", 1000, &control::poseCallback,this);

    wii_communication_sub = nh_.subscribe<std_msgs::Int16MultiArray>\
	("wii_communication",1000,&control::wiiCommunicationCallback,\
	 this);
    
    R1 = Rect( Point(28.9, 22.6), Point(35.3, 18.9));
    R2 = Rect( Point(31.8, 10.3), Point(40.3, 17.1));
    R3 = Rect( Point(22.7, 35.0), Point(30.5, 12.6));
    R4 = Rect( Point(20.2, 0.00), Point(27.9, 21.9));
    
    rect_msg = rectsToPoseArray(R1, R2, R3, R4);
    speed_gain_msg.data = 1.0; /* Initially no speed gain */    
 
    isInBoostArea = false;
    speedGainFactor = 1.5;
}

// Subscribe to the local planner and map the steering angle 
// (and the velocity-but we dont do that here-) 
// to pulse width modulation values.
void control::cmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	if (control_Mode.data == 0 || BOOST_MODE_OFF){
	   /* In manual mode, speed boost should be off */ 
	   speed_gain_msg.data = 1.0;
	   ROS_INFO("[Boost] Disabled by definition or C-Button");
	} else {
	   /* Autonomous mode: Activate boost functionality */
	   if (msg != 0){
	      if(isInBoostArea){
		 speed_gain_msg.data = speedGainFactor;
		 ROS_INFO("[Boost] Inside:Factor:%f", speedGainFactor);
              } else if (!isInBoostArea){
                 speed_gain_msg.data = 1.0;
		 ROS_INFO("[Boost] Is NOT in area");
		}
	    }
	}
}

void control::wiiCommunicationCallback(const std_msgs::Int16MultiArray::ConstPtr& msg)
{
    control_Mode.data = msg->data[0];
}


/* This part distinguishes wether car is in boost area or not.
   It does not have to subscribe to PoseStamped, but can stay
   like this for now. */
void control::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    /* Transform the received pose to the map frame: */
    tf::StampedTransform transform;
    std::string map_frame("/map");
    std::string base_frame("/base_link");
  
    bool tf_ok = true;
    try {
	control::listener.lookupTransform\
		(map_frame, base_frame, ros::Time(0), transform);
    } catch(tf::TransformException ex) {
	tf_ok = false;
        ROS_WARN(" %s", ex.what());
    }
    
    if(tf_ok) {

	Point p(transform.getOrigin().getX(), \
		transform.getOrigin().getY() );
	if(R1.isInside(p) || R2.isInside(p) || \
	   R3.isInside(p) || R4.isInside(p)){
	   // ROS_INFO\
		("Is in Boost area. Pos: (%f|%f)",p.x, p.y);
	    isInBoostArea = true;
        } else {
	   // ROS_INFO\
		("Is not in Boost area. Pos: (%f|%f)",p.x, p.y);
	    isInBoostArea = false;
	}
    } else {
	isInBoostArea = false;
    } 
}

/* Generates a PoseArray. The poses in the array describe the rects
   that are given by r1-r4. */
geometry_msgs::PoseArray control::rectsToPoseArray(Rect r1, Rect r2, \
						   Rect r3, Rect r4){
	ROS_INFO("Call: rectsToPoseArray");
	unsigned p = 0; //Index for poses
	geometry_msgs::PoseArray posearray;
	posearray.header.stamp = ros::Time::now();
	posearray.header.frame_id = "/map"; //"/world"
	posearray.poses.resize(4*2*3);//4rects@2points@3poses
	std::vector<Rect> rectarray(4);
	rectarray[0] = r1;
	rectarray[1] = r2;
	rectarray[2] = r3;
	rectarray[3] = r4;
	
	/* Allocate space for 6 poses of a rect */
	boost::array<geometry_msgs::Pose, 6> poses;

	for(unsigned i = 0; i < 4; ++i){
	   poses = rectarray[i].asPoses();
	   for(unsigned j = 0; j < 6; j++){
		posearray.poses[p++] = poses[j];
	   }
	}

	return posearray;
}

//geometry_msgs::PoseArray control::rectsToPoseArray(Rect r1, Rect r2, \
//						   Rect r3, Rect r4){
//	geometry_msgs::PoseArray posearray;
//	posearray.header.stamp = ros::Time::now();
//	posearray.header.frame_id = "/map"; //"/world"
//	posearray.poses.resize(8);
//	std::vector<Rect> rectarray(4);
//	rectarray[0] = r1;
//	rectarray[1] = r2;
//	rectarray[2] = r3;
//	rectarray[3] = r4;
//	unsigned i,j;
//	for (i=j=0; i < posearray.poses.size(); i+=2, j++){
//	   posearray.poses[i].position.x = rectarray[j].p1.x;
//	   posearray.poses[i].position.y = rectarray[j].p1.y;
//	   posearray.poses[i].position.z = 0;
//	   posearray.poses[i].orientation.x = 0;
//	   posearray.poses[i].orientation.y = 0;
//	   posearray.poses[i].orientation.z = 0;
//	   posearray.poses[i].orientation.w = 0;
//
//	   posearray.poses[i+1].position.x = rectarray[j].p2.x;
//	   posearray.poses[i+1].position.y = rectarray[j].p2.y;
//	   posearray.poses[i+1].position.z = 0;
//	   posearray.poses[i+1].orientation.x = 0;
//	   posearray.poses[i+1].orientation.y = 0;
//	   posearray.poses[i+1].orientation.z = 0;
//	   posearray.poses[i+1].orientation.w = 0;
//	   ROS_INFO("POSE TRANSFORM %d", i);
//
//	}
//
//	return posearray;
//}
