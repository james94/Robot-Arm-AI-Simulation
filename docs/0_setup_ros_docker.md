# Setup ROS Unity Docker Dev Environment

## Build ROS Docker Image & Launch Docker Container 

1\. Build Pose Estimation ROS Unity Docker Image

~~~bash
cd Robot-Arm-AI-Simulation/

cd docker/unity-robotics/
docker build -t unity-robotics:pose-estimation -f Dockerfile .
~~~

2\. Launch Pose Estimation ROS Docker Container

~~~bash
docker run -it --rm -p 10000:10000 -p 5005:5005 unity-robotics:pose-estimation /bin/bash
~~~

## Launch ROS MoveIt in Docker Container

3\. Source ROS workspace

~~~bash
source devel/setup.bash
~~~

4\. Start the Roscore, set the ROS parameters, start the server endpoint, start the Mover Service and Pose Estimation nodes and launch MoveIt:

~~~bash
roslaunch ur3_moveit pose_est.launch 
~~~

This launch file loads all ROS related files, starts ROS nodes required for trajectory planning for the UR3 robot. 

You can return to the main README.md at root of our project and continue on in section: "**Play Unity UR Robot Arm Grip Object Prediction**".

## References

For more information, reference the following tutorial series because in the description,
they provide the link to download their assets including Dockerfiles to build
their docker image.

Later we will have our own docker image.

- Robotics Development: ROS, Unity3D, DL & DevOps Tools (Tutorial Series): https://youtube.com/playlist?list=PLB8VXMjsTRoue4aodor1lDyZhFTKEP58i&si=e_JukXpAezD40sQt

- For more information from the original project, refer to Unity-Technologies - Robotics-Object-Pose-Estimation: https://github.com/Unity-Technologies/Robotics-Object-Pose-Estimation/tree/main

