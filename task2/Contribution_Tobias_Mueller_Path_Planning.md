# Contribution Tobias Müller: Path Planning for Slalom Task

The contribution is part of the node "CP.py" of the second task. Please read the [Documentation](task2/README.md) of the task for general information about the node.

The functionality is implemented in the function "calculatePath" of the second task node. 

Program Flow of the function:
  1. Waits until the planning position was reached and a scan signal and amcl pose was received.
  2. Tries to recognize at least two pylons and validates their position (distance to the wall, distance eachother)
  3. The first pylon will be the starting point of further calculations:
   1. With an offset of the starting pylon the goals between each two pylons of the path are calculated
   2. For better path planning we need also goals on the left or right side of the pylons.
  4. The calculated goals will be published.

## Execution

The functions will be directly used by the node of the second task. No separated execution is required.

## Video

The following video shows the working code:

https://www.dropbox.com/s/1tjf8q5szqpcldp/Group%2001%20-%20Slalom%20task.mp4?dl=0
