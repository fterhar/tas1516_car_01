# Contribution Fynn Terhar: A node that assists in training a route to the robot.

The contribution consists of one node "tas_waypoints" It consists of all files in this directory. tas_waypoints.cpp starts the node.

This contribution addresses the goal to let the robot go to previously trained waypoints. The storing of the waypoints is describes seperatly in [Training a path to the robot](tas_waypoints/README_Record.md). Once the waypoints are recorded, this node can also replay them, to make the robot DO what is previously was TAUGHT. In order to achieve this, a boost thread was implemented in this node. It continuously runs and checks if the button state changed. If the C-Button is pressed, the thread enters replay mode. During this mode, all stored waypoints are navigated to one after the other, as long as the C-Button is pressed. It is also possible to record new waypoints during replaying them. The car navigates to the waypoints by using ROS Action API and respective feedback methods.

=================================================================

Program Flow:

1. When starting the node, it first establishes a connection to a running movebase_server. If the server is not running yet, it retries until success.

2. The program will now be in recording mode. Whenever the user presses the A-Button on the Wii-Mote, the node will store a waypoint object in a dynamic vector. The waypoint is set at the position where the robot is on the map. The programm will stay in this recording mode, until the user presses C-Button on the Nunchuk. It then enters Replay-Mode.

3. Replay mode is active as long as C-Button is pressed. In this mode, the robot tries to iteratively reach all stored waypoints one after the other. The replay mode can always be left by releasing C-Button. The node will again be in Recording mode. The user can add new waypoints on the fly and return to replay mode again and continue.

## Execution

Simply run the tas_waypoints node. It is important to have a respective movebase_server running. The tas_waypoints node will be waiting on a movebase_server to be ready before it can fully function. It can be closed by Ctrl-C as usual.



