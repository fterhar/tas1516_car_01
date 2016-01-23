#!/usr/bin/python
# -*- coding: utf-8 -*-

# Run with: rosrun task2 CP.py

import os
import math
import numpy

import ConfigParser

#
# ROS-specific imports
#
import rospy

import actionlib
from actionlib_msgs.msg import *

from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from tf.transformations import euler_from_quaternion, quaternion_from_euler

# 
# Configuration file: Contains all hardcoded stuff like initional position, ...
# 
global CONFIG_FILE
CONFIG_FILE = "parameters.ini"

# 
#  Initialization: Variables and Action Server for setting goales
# 
def init():
    rospy.loginfo("Initialization")
    # Variables
    global amcl_pose_received
    amcl_pose_received = False
    global planned
    planned = False
    global ranges
    ranges = []

    # Calculates the orientation only once
    global orientation
    orientation = quaternion_from_euler(0, 0, -90 * math.pi / 180)
    
    # Action Server
    global move_base
    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")
    # Wait 60 seconds for the action server to become available
    move_base.wait_for_server(rospy.Duration(60))
    rospy.loginfo("Connected to move base server")

# 
# Setting the initional position
# 
def pubInitLocation():
    global orientation

    rate = rospy.Rate(100) # 100 Hz
    pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
    
    while not rospy.is_shutdown():
        # Create 6x6 covariance matrix as list with 36 entries
        cov = []
        for i in range(0,36):
            cov.append(0)
        # Set values of covariance matrix --> other values of matrix are 0
        cov[0] = 1
        cov[7] = 1
        cov[35] = 3

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = rospy.get_rostime()
        msg.pose.pose.position.x = config.getfloat('InitPosition', 'x')
        msg.pose.pose.position.y = config.getfloat('InitPosition', 'y')
        msg.pose.pose.position.z = config.getfloat('InitPosition', 'z')
        msg.pose.pose.orientation.x = orientation[0]
        msg.pose.pose.orientation.y = orientation[1]
        msg.pose.pose.orientation.z = orientation[2]
        msg.pose.pose.orientation.w = orientation[3]
        msg.pose.covariance = cov
        
        pub.publish(msg)
        rospy.loginfo('Initional position was set')
        break
        rate.sleep()

# 
# Callback received by topic "amcl_pose"
# The msg contains the estimated position of the car
# http://wiki.ros.org/amcl
# 
def amcl_pose_callback(msg):
    global amcl_pose_received
    global amcl_pose
    amcl_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
    amcl_pose_received = True

# 
# Callback received by topic "scan"
# The msg contains the laser scan information of the lidar system
# http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
# 
def scan_callback(msg):
    # range data [m] (Note: values < range_min or > range_max should be discarded)
    global ranges
    ranges = msg.ranges
    # angular distance between measurements [rad]
    global increment
    increment = msg.angle_increment
    # start angle of the scan [rad]
    global angle_min
    angle_min = msg.angle_min

# 
# Sets the planning position for futher path planning
# The start position is bad for planning, because it is not possible
# 
def setPositionForPlanning():
    global config
    global move_base
    global orientation
    
    rospy.loginfo("Set Planning Position Goal")
    # Intialize the goal
    goal = MoveBaseGoal()
    # Use the map frame to define goal poses
    goal.target_pose.header.frame_id = 'map'
    # Set the time stamp to "now"
    goal.target_pose.header.stamp = rospy.Time.now()
    # Set the goal pose
    goal.target_pose.pose.position.x = config.getfloat('PositionForPlanning', 'x')
    goal.target_pose.pose.position.y = config.getfloat('PositionForPlanning', 'y')
    goal.target_pose.pose.position.z = config.getfloat('PositionForPlanning', 'z')
    goal.target_pose.pose.orientation.x = orientation[0]
    goal.target_pose.pose.orientation.y = orientation[1]
    goal.target_pose.pose.orientation.z = orientation[2]
    goal.target_pose.pose.orientation.w = orientation[3]
    
    # Set goal and ...
    move_base.send_goal(goal)
    # ... wait one minute to reache the goal
    finished_within_time = move_base.wait_for_result(rospy.Duration(60)) 
    # If we don't get there in time, abort the goal
    if not finished_within_time:
        move_base.cancel_goal()
        rospy.logerr("Timed out achieving goal")
    else:
        state = move_base.get_state()
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("Reached Planning position!")

# 
# If the car reached the planning position, we can start to plan the path through the slalom
# 
def calculatePath():
    global config
    # Only plan once
    global planned

    # Received by the amcl_pose topic
    global amcl_pose_received
    global amcl_pose
    
    # Received by the scan topic
    global ranges
    global increment
    global angle_min

    # Get config values:
    # The values are received by the config file
    validate_planning_position = config.getfloat('Planning', 'yCoordLimit')
    num_pylons = config.getint('Map', 'NumPylons')
    tolerance = config.getfloat('Planning', 'Tolerance')
    min_distance_wall = config.getfloat('Planning', 'MinDistanceWall')
    width_pylon = config.getfloat('Map', 'EdgePylons')
    distance_goal = config.getfloat('Planning', 'DistanceGoalToPylon')
    radius_pylon = config.getfloat('Map', 'RPylons')

    while not rospy.is_shutdown():

        # Validate that the planning position was reached
        if amcl_pose_received and amcl_pose[1] < validate_planning_position:
            if planned == False and len(ranges) > 0:
                pylons_counted = 0
                goals_for_path = [
                    (0, 0),
                    (0, 0),
                    (0, 0),
                    (0, 0),
                    (0, 0),
                    (0, 0),
                    (0, 0),
                    ]

                pylons_distance = []

                # 
                # Iterate over laser scans
                #
                i = 1
                pylons_increment = []
                while i < len(ranges) - 1:
                    # Infinity or faulty data?
                    if ranges[i + 1] < ranges[i] - min_distance_wall and ranges[i] < 50:
                        rospy.loginfo("Pylon candidate with distance: %f m" % ranges[i + 1])

                        # How width is the object?
                        width_based_on_increment =  math.atan(width_pylon / (2 * ranges[i + 1]) * 2) // increment
                        static_i_next = i + 1
                        while ranges[static_i_next] < ranges[i + 1] + tolerance and ranges[static_i_next] > ranges[i + 1] - tolerance and i <= static_i_next + width_based_on_increment and i < len(ranges) - 1:
                            i += 1

                        if i <= static_i_next + width_based_on_increment and (ranges[i + 1] + ranges[i + 2] + ranges[i + 3] + ranges[i + 4]) / 4 > ranges[static_i_next] + min_distance_wall:
                            pylons_distance.append(ranges[static_i_next + (i - static_i_next) // 2])
                            pylons_increment.append(increment * 180 / math.pi * (static_i_next + (i - static_i_next) // 2))
                            pylons_counted += 1
                            rospy.loginfo("Pylon candidate selected")

                        # Too width?
                        if not i <= static_i_next + width_based_on_increment:
                            rospy.logerr('Width too large: %f m' % (2 * ranges[static_i_next] * math.tan((i - static_i_next) * increment / 2)))

                        # Check distance to wall
                        if not (ranges[i + 1] + ranges[i + 2] + ranges[i + 3] + ranges[i + 4]) / 4 > ranges[static_i_next] + min_distance_wall:
                            rospy.logerr('Distance to wall not enough: Distance %f m' % ((ranges[i + 1] + ranges[i + 2] + ranges[i + 3] + ranges[i + 4]) / 4))

                        i += 1
                    else:
                        i += 1

                location_of_pylons = []
                if pylons_counted > 1 and pylons_counted < 5:
                    rospy.loginfo('%d pylons recognized' % pylons_counted)

                    i = 0
                    while i < pylons_counted:
                        angles = euler_from_quaternion((0, 0, amcl_pose[2], amcl_pose[3]))
                        a = pylons_increment[i] + angles[2] * 180 / math.pi + angle_min * 180 / math.pi
                        offset_to_center = (radius_pylon / pylons_distance[i]) + 1

                        # What is the location of the pylons?
                        location_of_pylons.append((
                            offset_to_center * pylons_distance[i] * math.cos(a * math.pi / 180) + amcl_pose[0], 
                            offset_to_center * pylons_distance[i] * math.sin(a * math.pi / 180) + amcl_pose[1]
                            ))

                        i += 1

                    # 
                    # Validation:
                    # Is the distance between the pylons plausible?
                    # 
                    i = 0
                    avg_pylon_distance = 0
                    while i < pylons_counted - 1:
                        # Distance between the actual pylon and the next one
                        distance = numpy.sqrt((location_of_pylons[i][0] - location_of_pylons[i + 1][0]) ** 2
                                       + (location_of_pylons[i][1] - location_of_pylons[i + 1][1]) ** 2)
                        # Average distance
                        avg_pylon_distance = avg_pylon_distance - (avg_pylon_distance - distance) / (i + 1)
                        i += 1

                    if avg_pylon_distance <= 2 and avg_pylon_distance >= 1:
                        rospy.loginfo('Average distance between pylons: %f m' % avg_pylon_distance)

                        pylons_position_to_each_other = []
                        # Get the nearest pylon
                        nearest_pylon =  pylons_distance.index(min(pylons_distance))
                        if nearest_pylon > 0:
                            pylons_position_to_each_other = (location_of_pylons[nearest_pylon-1][0] - location_of_pylons[nearest_pylon][0], location_of_pylons[nearest_pylon-1][1] - location_of_pylons[nearest_pylon][1])
                        else:
                            pylons_position_to_each_other = (location_of_pylons[nearest_pylon+1][0] - location_of_pylons[nearest_pylon][0], location_of_pylons[nearest_pylon+1][1] - location_of_pylons[nearest_pylon][1])

                        # Right order? The first pylon is closer then 1.2 m (planner position)
                        if pylons_distance[nearest_pylon] > 1.2:
                            rospy.logwarn('Reorder first and second pylon, because the distance is too large')
                            rospy.logwarn('Distance to nearest pylon: %f m' % pylons_distance[nearest_pylon])
                            first_pylon_loc = (location_of_pylons[nearest_pylon][0] - pylons_position_to_each_other[0], location_of_pylons[nearest_pylon][1] - pylons_position_to_each_other[1])
                        else:
                            first_pylon_loc = location_of_pylons[nearest_pylon]

                        move_parallel_to_pylon = (
                            (distance_goal / numpy.sqrt(pylons_position_to_each_other[0] ** 2 + pylons_position_to_each_other[1] ** 2)) * pylons_position_to_each_other[1], 
                            (distance_goal / numpy.sqrt(pylons_position_to_each_other[0] ** 2 + pylons_position_to_each_other[1] ** 2)) * -pylons_position_to_each_other[0]
                            )

                        # Set goals on the left/right side
                        goals_for_path[1] = (first_pylon_loc[0]
                                        + pylons_position_to_each_other[0]
                                        + move_parallel_to_pylon[0],
                                        first_pylon_loc[1]
                                        + pylons_position_to_each_other[1]
                                        + move_parallel_to_pylon[1])
                        goals_for_path[3] = (first_pylon_loc[0] + 2
                                        * pylons_position_to_each_other[0]
                                        - move_parallel_to_pylon[0],
                                        first_pylon_loc[1] + 2
                                        * pylons_position_to_each_other[1]
                                        - move_parallel_to_pylon[1])
                        goals_for_path[5] = (first_pylon_loc[0] + 3
                                        * pylons_position_to_each_other[0]
                                        + move_parallel_to_pylon[0]
                                        + 0.05, first_pylon_loc[1] + 3
                                        * pylons_position_to_each_other[1]
                                        + move_parallel_to_pylon[1])

                        # Set goals between the pylons
                        goals_for_path[0] = (first_pylon_loc[0]
                                        + pylons_position_to_each_other[0] / 2,
                                        first_pylon_loc[1]
                                        + pylons_position_to_each_other[1] / 2)
                        goals_for_path[2] = (first_pylon_loc[0] + 3
                                        * pylons_position_to_each_other[0] / 2,
                                        first_pylon_loc[1] + 3
                                        * pylons_position_to_each_other[1] / 2)
                        goals_for_path[4] = (first_pylon_loc[0] + 5
                                        * pylons_position_to_each_other[0] / 2,
                                        first_pylon_loc[1] + 5
                                        * pylons_position_to_each_other[1] / 2)
                        goals_for_path[6] = (first_pylon_loc[0] + 9
                                        * pylons_position_to_each_other[0] / 2,
                                        first_pylon_loc[1] + 9
                                        * pylons_position_to_each_other[1] / 2)

                        rospy.loginfo('#########################')
                        rospy.loginfo('Planned Goals: %s' % str(goals_for_path))
                        rospy.loginfo('#########################')

                        i = 0
                        while i < 7:
                            goal_quaternion = quaternion_from_euler(0, 0, -94 * math.pi / 180)

                            # Intialize the goal
                            goal = MoveBaseGoal()
                            # Use the map frame to define goal poses
                            goal.target_pose.header.frame_id = 'map'
                            # Set the time stamp to "now"
                            goal.target_pose.header.stamp = rospy.Time.now()

                            goal.target_pose.pose.position.x = goals_for_path[i][0]
                            goal.target_pose.pose.position.y = goals_for_path[i][1]
                            goal.target_pose.pose.position.z = 0
                            goal.target_pose.pose.orientation.x = goal_quaternion[0]
                            goal.target_pose.pose.orientation.y = goal_quaternion[1]
                            goal.target_pose.pose.orientation.z = goal_quaternion[1]
                            goal.target_pose.pose.orientation.w = goal_quaternion[2]

                            move_base.send_goal(goal)
                            # Allow 1 minute to get there
                            finished_within_time = move_base.wait_for_result(rospy.Duration(60)) 
                            # If we don't get there in time, abort the goal
                            if not finished_within_time:
                                move_base.cancel_goal()
                                rospy.logerr('Timed out achieving goal %d' % i)
                            else:
                                state = move_base.get_state()
                                if state == GoalStatus.SUCCEEDED:
                                    rospy.loginfo('Reached goal position %d' % i)
                            i += 1

                        if planned == False:
                            planned = True
                    else:
                        rospy.logerr('The average distance is too high (>2m) or too low (<1m)')
                else:
                    rospy.logerr('The number of pylons is not valid (less 2 or more then 4): %d' % pylons_counted)

    rospy.spin()


if __name__ == '__main__':
    try:
        # Init Node
        rospy.init_node('CP', anonymous=True)
        
        # Read Config File
        rospy.loginfo('Read Config File parameters.ini')
        global config
        config = ConfigParser.ConfigParser()
        global CONFIG_FILE
        config.read(os.path.join(os.path.abspath(os.path.dirname(__file__)), CONFIG_FILE))
        
        # Init global Variables
        init()
        # Set the starting position
        pubInitLocation()
        
        # Subscribe and register Callbacks
        rospy.Subscriber('scan', LaserScan, scan_callback)
        rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, amcl_pose_callback)

        # Set Position for Planning
        setPositionForPlanning()
        # Calculates the Path
        calculatePath()
    except rospy.ROSInterruptException:
        pass
