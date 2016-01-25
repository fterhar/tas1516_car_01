# Contribution Fynn Terhar: A node that defines areas that boost up the robot speed

The contribution consists of one node "tas_speed_gain". It defines areas on the map using rectangles. It measures the current position of the robot, to detect if it is in a defined rectangle. The was designed to have a good speed on the straight parts of the hallway.

Program Flow:
1. On execution the node defines 4 rectangles, this is hardcoded so far, but this can be made dynamic by using some logic when it is required. The rectangles are implemented as an own class. A retangle is described by two diagonal points. The points are also implemented in an own class. A point is described as a 2D x,y coordinate. The node also publishes a Pose_Array. This is intended to be subscribed in rviz. The Posearray will vizualize the rectangles, which makes testing very easy in rviz and a simulator like gazebo.
2. The node constantly publishes a scalar "speed_gain_factor", which is 1.0 most of the time. Only then the robot is detected to be inside of a rectangle, the speed_gain_factor will be greater than 1.0. For example a speed_gain_factor of 2.0 will double the robots total speed.
3. The published speed_gain_factor is received by tas_autonomous_control, where it is processed further so that the PWM output will cause the robot to have the set speed_gain_factor. The speed gain will only be active when the robot is in autonomous mode. 
4. The contribution can also be used for different things other than speed_gain. For example the rectangles could be used to define "No entry zones" where the robot shall never go, if detected inside anyways, some measures can be taken.

## Execution

Simply run the tas_speed_gain node. It requires the wii_mote to be running as well as the tas_autonomous_control node.  It can be closed by Ctrl-C as usual.


## Video

The following video shows the working code:

-- TO BE ADDED --
