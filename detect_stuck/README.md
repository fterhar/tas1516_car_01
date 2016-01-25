# Contribution Anthony Carnevale: Move backwards if next goal can't be reached

The contribution consists of writing the node "detect_stuck" which receives the needed velocity of the "move_base" node and the current velocity of the adapted "tas_odom" node and compares both messages to decide if the car got stuck and a short backwards move is needed for a new try to reach the goal.

The written/changed code is in the files: 
- https://github.com/tas1516-group-01/tas1516_car_01/blob/master/detect_stuck/src/control_vel.cpp
- https://github.com/tas1516-group-01/tas1516_car_01/blob/master/tas_odom/src/tas_odom.cpp
- https://github.com/tas1516-group-01/tas1516_car_01/blob/master/publish_short_moves/src/short_moves_pub.cpp

Program Flow:
  1. Initialization: Publish current velocity in tas_odom and create callback functions in detect_stuck for receiving the velocity values.
  2. What tas_odom was changed for: tas_odom receives the current position of hector mapping and based on a suitable time delay between the current and the old position data it calculates the current velocity of the car. This is sent to the topic odom.
  3. What detect_stuck does: It receives both, the current velocity of the mentioned tas_odom node and the needed velocity based on the topic "cmd_vel". It contains the needed velocity when goal is set. For example: if the goal is in front of the car the value is 1.0. Another fact is that when a goal is set the angular velocity in z-orientation will never be zero since the car tries to steer in the direction where the goal is. That's the point to be picked. If the current velocity is 0 but the steering angle is not 0, it means that the car got stuck (at least this is the case when it happens for a longer time, therefore I don't work with the absolute velocity value but with the mean value of the last receivings and check for a lower bound to be passed). If the car got stuck the node publish_short_moves is executed. Here the ACTIVE part of the code is relevant, the uncommented part is needed for the localization (explained in my other contribution).

## Execution

Starting the node:

First start tas_odom for publishing the current velocity

> rosrun tas_odom tas_odom

Now start detect_stuck

> rosrun detect_stuck control_vel

## Video

The following video shows the working code:

https://www.dropbox.com/s/1o88bqgkqfuoumy/detect_stuck_video.MOV?dl=0
