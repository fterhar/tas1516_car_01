# Documentation second task: Slalom

The second task consists of one node "CP.py". The code is in the file: https://github.com/tas1516-group-01/tas1516_car_01/task2/src/CP.py

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

The configuration files (parameters for planner, etc.) should be the standard values. The only parameter, which have to be modified, is the value of yaw_goal_tolerance (new value: 6.28). The parameter is important for the decision, whether it has reached the goal with the right orientation.

Starting the node:

> rosrun task2 CP.py

## Video

The following video shows the working code:

https://www.dropbox.com/s/cvabxxo1geeikb7/Second_Task_Slalom.mp4?dl=0
