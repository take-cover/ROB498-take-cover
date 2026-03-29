#!/bin/bash

# Record ROS2 bag data from take_cover/aruco/debug_image topic and save it to a bag file named "aruco_debug_image_bag"
ros2 bag record /take_cover/aruco/debug_image -o aruco_debug_image_bag