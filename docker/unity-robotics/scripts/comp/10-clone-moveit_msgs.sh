#!/bin/bash

mkdir -p $ROS_WORKSPACE/src

git clone https://github.com/ros-planning/moveit_msgs.git $ROS_WORKSPACE/src/moveit_msgs

if [ $? -eq 0 ]; then
    echo "Cloned moveit_msgs repository"
else
    echo "Failed to clone moveit_msgs repository"
    exit 1
fi