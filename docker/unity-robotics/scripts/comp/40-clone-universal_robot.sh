#!/bin/bash

git clone https://github.com/ros-industrial/universal_robot $ROS_WORKSPACE/src/universal_robot

if [ $? -eq 0 ]; then
    echo "Cloned universal_robot repository"
else
    echo "Failed to clone universal_robot repository"
    exit 1
fi
