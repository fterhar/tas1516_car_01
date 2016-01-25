# Contribution Tobias Müller: One combined scanner

The contribution consists of one node "CP.py", one launch file, and multiple configuration changes. The code is in the file: https://github.com/tas1516-group-01/tas1516_car_01/task2/src/CP.py

Program Flow:
  1. Initialization: reads the config file, starts the action server, sets the initional position (start position 2) of the car, and publishes the position to the topic "initialpose"
  2. After starting the action server the robot can receive goals. The first goal is the planner position. The position is important, because the robot needs an overview over the whole obstacle for the planning task. The starting position is a bad position for planning, because only the first pylon is visible by the front scanner.
  3. If the robot reach the planning position (left side of the first pylon), it will start with the planning task:
  	1. Recognition of the pylons and their distance to the car
  	2. Validation of the detected pylons (valid distance, distance to wall, distance one pylon to another)
  	3. Setting goals: seven goals will be set: Between the pylons and left/right side of the pylons
  	4. The integrated planner of the car will plan the path through the obstacle.
  4. After planning the goals the car will start driving through the obstacle.

## Execution

Before starting the node it is important to setup the configuration for hector_mapping and amcl. Per default this nodes will use the topic "scan" for their algorithms.

Add in the move_base.launch file at the end of the specification the following line:

`<!-- Run AMCL -->`

`<node pkg="amcl" type="amcl" name="amcl" respawn="true">`

`  	...`

`  	<remap from="scan" to="/two_scan"/>`

`</node>`

Now we have to change the scan topic in the hector_mapping node:

`<!-- hector mapping -->`

`<node pkg="hector_mapping" type="hector_mapping" name="hector_mapping" output="screen">`

`  	<!-- Default: <param name="scan_topic" value="scan" /> -->`

`   <param name="scan_topic" value="/two_scan" />  <!-- Added -->`

`   ...`

`</node>`

The back scanner is using a own coordinate system. So we have to add the following line in the odom.launch file.

`<!-- Added: the back scanner requires a own coordiante system -->`

`<node pkg="tf" type="static_transform_publisher" name="base_link_to_laser_back" args="-0.28 0.0 0.21 3.14 0 0 /base_link /laser_back 40" />`

Starting the node:

> rosrun two_scans two_scans_node

Starting the converter for the pointcloud:

> roslaunch two_scans startCloudtoLaser.launch

The converter will produce a well formed laserscan message, which is required by the amcl node.

## Video

The following video shows the working code:

https://www.dropbox.com/s/bk8pposgng5lebi/Contribution_Tobias_Mueller_One_combined_scanner.mp4?dl=0
