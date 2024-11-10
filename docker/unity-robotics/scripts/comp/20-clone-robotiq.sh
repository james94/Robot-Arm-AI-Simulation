#!/bin/bash

git clone -b noetic-mods https://github.com/JStech/robotiq.git $ROS_WORKSPACE/src/robotiq

if [ $? -eq 0 ]; then
    echo "Cloned robotiq repository"
else
    echo "Failed to clone robotiq repository"
    exit 1
fi
