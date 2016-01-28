# Contribution Tobias Müller: One combined scanner

The contribution consists of one node "two_scans_node", one launch file, and multiple configuration changes. The code is in the file: https://github.com/tas1516-group-01/tas1516_car_01/blob/master/two_scans/src/two_scans_node.cpp

Program Flow:
  1. Initialization: Subscribe the front and back scan topic, publish combined scan signal and sets the listener for the transformation.
  2. For the processing of both scanner signals two callback functions (front and back scanner) are required. The callback functions will receive the scan information and processes the coordinate system transformation from the laser into the target frame of the combined scanner. Then the signal will be converted into a pointcloud and be saved into class varibale for later usage.
  3. If both scanner signals were received and the time difference of the signals are less then 20 ms, a combined scan signal is published. Otherwise only the front scanner signal will be published. A separated submission of both signals is not possible, because the simulated combined scanner requires a constant timestep between two signals.

## Execution

Before starting the node it is important to set up the configuration for hector_mapping and amcl. Per default this nodes will use the topic "scan" for their algorithms.

Add at the end of the specification of the move_base.launch file the following line:

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

Installing and starting the converter for the pointcloud:

> (optional) sudo apt-get install ros-indigo-pointcloud-to-laserscan

> roslaunch two_scans startCloudtoLaser.launch

The converter will produce a well formed laserscan message, which is required by the amcl node.

## Video

The following video shows the working code:

https://www.dropbox.com/s/bk8pposgng5lebi/Contribution_Tobias_Mueller_One_combined_scanner.mp4?dl=0
