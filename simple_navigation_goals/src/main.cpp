/**
 * This node sends fixed goals to move base via ROS Action API and receives feedback via callback functions.
 */

#include <iostream>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <actionlib/client/simple_action_client.h>

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

/**
 * Main function
 */
int main(int argc, char** argv){
    ros::init(argc, argv, "simple_navigation_goals"); // init and set name
    std::vector<geometry_msgs::Pose> waypoints; // vector of goals, with position and orientation

    geometry_msgs::Pose waypoint1;
      waypoint1.position.x =  23.8815116882
      waypoint1.position.y =  18.8573932648
      waypoint1.position.z =  0.0
      waypoint1.orientation.x =  0.0
      waypoint1.orientation.y =  0.0
      waypoint1.orientation.z =  -0.735896961989
      waypoint1.orientation.w =  0.677093539576
    waypoints.push_back(waypoint1);

    geometry_msgs::Pose waypoint2;
      waypoint2.position.x =  23.5099124908
      waypoint2.position.y =  15.2682342529
      waypoint2.position.z =  0.0
      waypoint2.orientation.x =  0.0
      waypoint2.orientation.y =  0.0
      waypoint2.orientation.z =  -0.708565325123
      waypoint2.orientation.w =  0.705645222497
    waypoints.push_back(waypoint2);

    geometry_msgs::Pose waypoint3;
      waypoint3.position.x =  23.4844474792
      waypoint3.position.y =  11.1678152084
      waypoint3.position.z =  0.0
      waypoint3.orientation.x =  0.0
      waypoint3.orientation.y =  0.0
      waypoint3.orientation.z =  -0.694711401216
      waypoint3.orientation.w =  0.719288585354
    waypoints.push_back(waypoint3);

    geometry_msgs::Pose waypoint4;
      waypoint4.position.x =  23.6034202576
      waypoint4.position.y =  7.79697990417
      waypoint4.position.z =  0.0
      waypoint4.orientation.x =  0.0
      waypoint4.orientation.y =  0.0
      waypoint4.orientation.z =  -0.748886861226
      waypoint4.orientation.w =  0.662697871645
    waypoints.push_back(waypoint4);

    geometry_msgs::Pose waypoint5;
      waypoint5.position.x =  23.2928962708
      waypoint5.position.y =  6.63507318497
      waypoint5.position.z =  0.0
      waypoint5.orientation.x =  0.0
      waypoint5.orientation.y =  0.0
      waypoint5.orientation.z =  -0.906841021846
      waypoint5.orientation.w =  0.421472847403
    waypoints.push_back(waypoint5);

    geometry_msgs::Pose waypoint6;
      waypoint6.position.x =  22.1547718048
      waypoint6.position.y =  5.53000259399
      waypoint6.position.z =  0.0
      waypoint6.orientation.x =  0.0
      waypoint6.orientation.y =  0.0
      waypoint6.orientation.z =  0.999413091741
      waypoint6.orientation.w =  0.0342559784155
    waypoints.push_back(waypoint6);

    geometry_msgs::Pose waypoint7;
      waypoint7.position.x =  19.078245163
      waypoint7.position.y =  5.87801170349
      waypoint7.position.z =  0.0
      waypoint7.orientation.x =  0.0
      waypoint7.orientation.y =  0.0
      waypoint7.orientation.z =  -0.99999991528
      waypoint7.orientation.w =  0.000411631617641
    waypoints.push_back(waypoint7);

    geometry_msgs::Pose waypoint8;
      waypoint8.position.x =  14.2898674011
      waypoint8.position.y =  5.99952316284
      waypoint8.position.z =  0.0
      waypoint8.orientation.x =  0.0
      waypoint8.orientation.y =  0.0
      waypoint8.orientation.z =  0.995585206001
      waypoint8.orientation.w =  0.0938621201159
    waypoints.push_back(waypoint8);

    geometry_msgs::Pose waypoint9;
      waypoint9.position.x =  10.8561782837
      waypoint9.position.y =  6.25628805161
      waypoint9.position.z =  0.0
      waypoint9.orientation.x =  0.0
      waypoint9.orientation.y =  0.0
      waypoint9.orientation.z =  0.836690372752
      waypoint9.orientation.w =  0.547676200089
    waypoints.push_back(waypoint9);

    geometry_msgs::Pose waypoint10;
      waypoint10.position.x =  10.2170791626
      waypoint10.position.y =  9.01545524597
      waypoint10.position.z =  0.0
      waypoint10.orientation.x =  0.0
      waypoint10.orientation.y =  0.0
      waypoint10.orientation.z =  0.669739461276
      waypoint10.orientation.w =  0.74259615809
    waypoints.push_back(waypoint10);

    geometry_msgs::Pose waypoint11;
      waypoint11.position.x =  10.8112611771
      waypoint11.position.y =  17.392665863
      waypoint11.position.z =  0.0
      waypoint11.orientation.x =  0.0
      waypoint11.orientation.y =  0.0
      waypoint11.orientation.z =  0.622659873485
      waypoint11.orientation.w =  0.782492608241
    waypoints.push_back(waypoint11);

    geometry_msgs::Pose waypoint12;
      waypoint12.position.x =  10.8818693161
      waypoint12.position.y =  19.3060493469
      waypoint12.position.z =  0.0
      waypoint12.orientation.x =  0.0
      waypoint12.orientation.y =  0.0
      waypoint12.orientation.z =  0.288743785587
      waypoint12.orientation.w =  0.957406406018
    waypoints.push_back(waypoint12);

    geometry_msgs::Pose waypoint13;
      waypoint13.position.x =  12.4265480042
      waypoint13.position.y =  19.6270713806
      waypoint13.position.z =  0.0
      waypoint13.orientation.x =  0.0
      waypoint13.orientation.y =  0.0
      waypoint13.orientation.z =  0.0183549890018
      waypoint13.orientation.w =  0.999831532999
    waypoints.push_back(waypoint13);

    geometry_msgs::Pose waypoint14;
      waypoint14.position.x =  16.2960968018
      waypoint14.position.y =  19.2553977966
      waypoint14.position.z =  0.0
      waypoint14.orientation.x =  0.0
      waypoint14.orientation.y =  0.0
      waypoint14.orientation.z =  -0.00853773959032
      waypoint14.orientation.w =  0.999963552837
    waypoints.push_back(waypoint14);

    geometry_msgs::Pose waypoint15;
      waypoint15.position.x =  21.2300453186
      waypoint15.position.y =  19.212348938
      waypoint15.position.z =  0.0
      waypoint15.orientation.x =  0.0
      waypoint15.orientation.y =  0.0
      waypoint15.orientation.z =  -0.0247131543948
      waypoint15.orientation.w =  0.99969458336
    waypoints.push_back(waypoint15);

    geometry_msgs::Pose waypoint16;
      waypoint16.position.x =  23.0693340302
      waypoint16.position.y =  19.2594299316
      waypoint16.position.z =  0.0
      waypoint16.orientation.x =  0.0
      waypoint16.orientation.y =  0.0
      waypoint16.orientation.z =  -0.29523019504
      waypoint16.orientation.w =  0.955426152006
    waypoints.push_back(waypoint16);

    geometry_msgs::Pose waypoint17;
      waypoint17.position.x =  24.2850532532
      waypoint17.position.y =  17.8909797668
      waypoint17.position.z =  0.0
      waypoint17.orientation.x =  0.0
      waypoint17.orientation.y =  0.0
      waypoint17.orientation.z =  -0.764020646483
      waypoint17.orientation.w =  0.645191794545
    waypoints.push_back(waypoint17);

    geometry_msgs::Pose waypoint18;
      waypoint18.position.x =  23.6596107483
      waypoint18.position.y =  16.7310581207
      waypoint18.position.z =  0.0
      waypoint18.orientation.x =  0.0
      waypoint18.orientation.y =  0.0
      waypoint18.orientation.z =  -0.66953927284
      waypoint18.orientation.w =  0.74277665696
    waypoints.push_back(waypoint18);

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
    return 0;
}
