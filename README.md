# Documentation

[Documentation second task: Slalom](task2/README.md)

=================================================================

### Contributions Tobias Müller

[Contribution Tobias Müller: One combined scanner](two_scans/README.md)

[Contribution Tobias Müller: Set Initional Pose and Planning Position for Slalom Task](task2/Contribution_Tobias_Mueller_Set_Initional_Pose_and_Planning_Position.md)

[Contribution Tobias Müller: Path Planning for Slalom Task](task2/Contribution_Tobias_Mueller_Path_Planning.md)

=================================================================

### Contributions Anthony Carnevale

[Contribution Anthony Carnevale: Execute short moves for better localization](publish_short_moves/README.md)

[Contribution Anthony Carnevale: Move backwards if next goal can't be reached](detect_stuck/README.md)

[Contribution Anthony Carnevale: Simulate the wii controller for testing waypoint recording](keyboard_sim/README.md)

additional contributions:

[Contribution Anthony Carnevale: Set initial_pose](publish_pose_init)

[Contribution Anthony Carnevale: Publish some hardcoded goals](publish_goals)

=================================================================

### Contributions Fynn Terhar

[Contribution Fynn Terhar: Speed gain for the straight hallways](tas_speed_gain/README.md)

[Contribution Fynn Terhar: Node for training a path to the robot](tas_waypoints/README.md)

tas_car
=======
Basic setup for the TAS stack

This repository has two branches: master and simulation.	
	1. Branch "master": for working on the real car
	2. Branch "simulation": for working on gazebo simulation

=================================================================

1. Branch "master":
	- Please follow the instruction on Moodle for better understanding about the package
	- This branch is only used for real car. It will not work on your own machine

=================================================================
2. Branch "simulation":
	- Provide a gazebo simulation model based on ackermann_vehicle package on ROS
	- Please follow the instructions in INSTALL.md to install all necessary packages
	- For launching simulator run:
		`roslaunch tas_simulator startSimulation.launch`
	- To start navigation stack in simulation run
		`roslaunch tas_simulator startNavigation.launch`	
	- To change to autonomous mode use
		`rostopic pub /wii_communication std_msgs/Int16MultiArray`
	hit "Tab" a few times and change that 0 to 1
	- And publish goals:
		`rosrun simple_navigation_goals simple_navigation_goals_node` 
   REMEMBER: This branch only provides you a gazebo simulation model, additional details and control strategies need to be added to fullfil your tasks
			
