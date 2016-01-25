# Contribution Anthony Carnevale: Simulate the wii controller for testing waypoint recording

The contribution consists of writing a simulation node which can be used for testing the waypoint recording in gazebo without wii controller.

The written/changed code is in the files: 
- https://github.com/tas1516-group-01/tas1516_car_01/blob/master/keyboard_sim/src/keyboard_sim_node.cpp

Program Flow:
  'a' or 'c' of the wiimote/state topic is set or reset by keyboard depending if recording waypoints or reaching them afterwards. The explaination of the waypoint recording is in the corresponding node README file. This node is helpful to simulate the functionality in gazebo.

## Execution

Starting the node:

> rosrun keyboard_sim keyboard_sim_node
