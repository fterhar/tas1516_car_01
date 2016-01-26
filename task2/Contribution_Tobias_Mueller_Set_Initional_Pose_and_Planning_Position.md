# Contribution Tobias Müller: Set Initional Pose and Planning Position for Slalom Task

The contribution is part of the node "CP.py" of the second task. Please read the [Documentation](task2/README.md) of the task for general information about the node.

The node contains two functions, which are part of this contribution: "pubInitLocation" and "setPositionForPlanning"

Program Flow:
  1. Initialization: A client instance of the SimpleActionServer is created.
  2. The function "pubInitLocation" sends a initional pose to the topic "initialpose". It is required for a better localization at the beginning.
  3. The function "setPositionForPlanning" will sends a goal to the Action Server and waits 60 seconds until the process will aborted. The planning position will be used to get a overview of the obstacle.

## Execution

The functions will be directly used by the node of the second task. No separated exectuion is required.

## Video

The following video shows the working code:

https://www.dropbox.com/s/cvabxxo1geeikb7/Second_Task_Slalom.mp4?dl=0
