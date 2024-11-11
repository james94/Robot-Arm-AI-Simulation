#!/bin/bash

git clone --recursive https://github.com/Unity-Technologies/Robotics-Object-Pose-Estimation.git $ROS_WORKSPACE/src/Robotics-Object-Pose-Estimation

if [ $? -eq 0 ]; then
    echo "Cloned Robotics-Object-Pose-Estimation repository"
else
    echo "Failed to clone Robotics-Object-Pose-Estimation repository"
    exit 1
fi

pushd $ROS_WORKSPACE/src/Robotics-Object-Pose-Estimation/ROS/src/

cp -r ur3_moveit $ROS_WORKSPACE/src/ur3_moveit

echo "Copied Unity's ur3_moveit package to ROS workspace"

cp -r ros_tcp_endpoint $ROS_WORKSPACE/src/ros_tcp_endpoint

echo "Copied Unity's ros_tcp_endpoint package to ROS workspace"

rm -rf $ROS_WORKSPACE/src/Robotics-Object-Pose-Estimation

echo "Removed Unity's Robotics-Object-Pose-Estimation package"

popd