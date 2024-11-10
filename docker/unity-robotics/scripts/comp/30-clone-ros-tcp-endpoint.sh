#!/bin/bash

git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git $ROS_WORKSPACE/src/ros_tcp_endpoint

if [ $? -eq 0 ]; then
    echo "Cloned ros_tcp_endpoint repository"
else
    echo "Failed to clone ros_tcp_endpoint repository"
    exit 1
fi
