/**
 * This node sends fixed goals to move base via ROS Action API and receives feedback via callback functions.
 */

#include <iostream>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <actionlib/client/simple_action_client.h>

#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16MultiArray.h"//TODO: Löschen?
#include <wiimote/State.h>
#include <math.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <cstdarg>
#include <vector>
#include <sensor_msgs/LaserScan.h>


using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/**
 * Callback function
 */
void doneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result) {
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
}

/**
 * Callback function, called once when the goal becomes active
 */
void activeCb() {
    ROS_INFO("Goal just went active");
}

/**
 * Callback function, called every time feedback is received for the goal
 */
void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) {
    ROS_INFO("[X]:%f [Y]:%f [W]: %f [Z]: %f", feedback->base_position.pose.position.x,feedback->base_position.pose.position.y,feedback->base_position.pose.orientation.w, feedback->base_position.pose.orientation.z);
}


float laserRange[719];
bool receivedlaser = false;

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& lasVal)
{
    for(int i = 0; i < 719; ++i)
    {
        laserRange[i] = lasVal.get()->ranges[i];
    }
    receivedlaser = true;
    for(static int i = 0; i < 719;i+=5)
    {
        ROS_INFO("%d:%f", i, laserRange[i]);
    }
}


/**
 * Main function
 */
int main(int argc, char** argv){
    ros::init(argc, argv, "slalom_node"); // init and set name
    std::vector<geometry_msgs::Pose> waypoints; // vector of goals, with position and orientation

    ros::NodeHandle nh_;
    ros::Subscriber laserValues = nh_.subscribe<sensor_msgs::LaserScan>("/scan",100,&laserCallback);
    
    
    tf::TransformListener listener;
    tf::StampedTransform transform;

    /* Transform the received pose to the map frame: */
    std::string map_frame("/map");
    std::string base_frame("/base_link");
	ros::Rate loop_rate(50);

    bool tf_ok = true;
     do
     {
        try {
            listener.lookupTransform\
		    (map_frame, base_frame, ros::Time(0), \
	     	 transform);
        } catch(tf::TransformException ex) {
	        tf_ok = false;
            ROS_ERROR(" %s", ex.what());
            loop_rate.sleep();
	        ros::spinOnce();
        }
    } while(!tf_ok && ros::ok());
    
    ROS_INFO("test");
	float startX = transform.getOrigin().getX();
	float startY = transform.getOrigin().getY();

/*	float startZ = transform.getOrigin().getZ();
	float startrx = transform.getRotation().getAxis().x();
	float startry = transform.getRotation().getAxis().y();
	float startrz = transform.getRotation().getAxis().z();
	*/
	float startrw = transform.getRotation().getAxis().w();
	
	//kurz vorwaerts fahren
    /*
	float nextX = transform.getOrigin().getX();
	float nextY = transform.getOrigin().getY();
	float nextZ = transform.getOrigin().getZ();
	float nextrx = transform.getRotation().getAxis().x();
	float nextry = transform.getRotation().getAxis().y();
	float nextrz = transform.getRotation().getAxis().z();
	float nextrw = transform.getRotation().getAxis().w();
	*/
	
	float mPylonLine = tan(startrw);//(nextY - startY) / (nextX - nextY);
	float xA = startY - mPylonLine * startX;
	
	ROS_INFO("m = %f/nxA = %f", mPylonLine, xA);
	
	int laserIndex = 0;
	int beginIndexP1 = 0;
	int endIndexP1 = 0;
	int cnt = 0;
	laserRange[0] = -1; /* Initially -1 */

	ROS_INFO("test2");
	while(ros::ok())  
	{
	    while(laserRange[0] == -1)
	    {
	            ROS_INFO("Waiting for laser data.");
	            loop_rate.sleep();
	            ros::spinOnce();
	    }
	    
	    if(cnt == 0)
	    {    
	        for(laserIndex = 0; laserIndex < 719; ++laserIndex)
	        {
	            if(laserIndex >= 719/3 && laserIndex <= 2*(719/3))
	            {
	                if(laserRange[laserIndex] <= 1.5 && beginIndexP1 == 0)
	                {
	                    beginIndexP1 = laserIndex;
	                }
	                else if(laserRange[laserIndex] > 1.5 && beginIndexP1 != 0)
                    {
                       endIndexP1 = laserIndex;
                       break;
                    }
	            }
	            
	        }
	
	        float rangeP1 = laserRange[(int) ((beginIndexP1 + endIndexP1)/2)] + 0.03;
	
	        ROS_INFO("beginIndex, %i endIndex, %i", beginIndexP1, endIndexP1);
	
	        ROS_INFO("Range to P1 = %f", rangeP1);
	
	
	        /*geometry_msgs::Pose waypoint;
	        waypoint.position.x    = px;
	        waypoint.position.y    = py;
	        waypoint.position.z    = pz;
	        waypoint.orientation.x = rx;
	        waypoint.orientation.y = ry;
	        waypoint.orientation.z = rz;
	        waypoint.orientation.w = rw;*/
            
            MoveBaseClient ac("move_base", true); // action client to spin a thread by default

            while (!ac.waitForServer(ros::Duration(5.0))) { // wait for the action server to come up
                ROS_INFO("Waiting for the move_base action server to come up");
            }

            move_base_msgs::MoveBaseGoal goal;
            goal.target_pose.header.frame_id = "map"; // set target pose frame of coordinates

            for(int i = 0; i < waypoints.size(); ++i) { // loop over all goal points, point by point
                goal.target_pose.header.stamp = ros::Time::now(); // set current time
                goal.target_pose.pose = waypoints.at(i);
                ROS_INFO("Sending goal");
                ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); // send goal and register callback handler
                ac.waitForResult(); // wait for goal result

                if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                    ROS_INFO("The base moved to %d goal", i);
                } else {
                    ROS_INFO("The base failed to move to %d goal for some reason", i);
                }
            }
            
        }
        cnt++;
        
        loop_rate.sleep();
    }
    return 0;
}

