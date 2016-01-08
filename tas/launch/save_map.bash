#!/bin/bash


cd ~/catkin_ws/src/tas1516_car_01/tas/launch/config/map_server
rosrun map_server map_saver -f map$RANDOM
