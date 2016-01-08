#!/bin/bash


roscd tas/launch/config/map_server
rosrun map_server map_saver -f map$RANDOM
