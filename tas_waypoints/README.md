# Contribution Fynn Terhar: A node that assists in training a route to the robot.

The contribution consists of one node "tas_waypoints" It consists of all files in this directory. tas_waypoints.cpp starts the node.

=================================================================

Program Flow:
1. When starting the node, it first establishes a connection to a running movebase_server. If the server is not running yet, it retries until success.

2. The program will now be in recording mode. Whenever the user presses the A-Button on the Wii-Mote, the node will store a waypoint object in a dynamic vector. The waypoint is set at the position where the robot is on the map. The programm will stay in this recording mode, until the user presses C-Button on the Nunchuk. It then enters Replay-Mode.

3. Replay mode is active as long as C-Button is pressed. In this mode, the robot tries to iteratively reach all stored waypoints one after the other. The replay mode can always be left by releasing C-Button. The node will again be in Recording mode. The user can add new waypoints on the fly and return to replay mode again and continue.

## Execution

Simply run the tas_waypoints node. It is important to have a respective movebase_server running. The tas_waypoints node will be waiting on a movebase_server to be ready before it can fully function. It can be closed by Ctrl-C as usual.



