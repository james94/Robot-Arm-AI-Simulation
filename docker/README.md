# Docker Image: Unity Robotics

1\. Build Pose Estimation ROS Docker Image

~~~bash
docker build -t unity-robotics:pose-estimation -f docker/unity-robotics/Dockerfile .
~~~

2\. Launch Pose Estimation ROS Docker Container

~~~bash
docker run -it --rm -p 10000:10000 -p 5005:5005 unity-robotics:pose-estimation /bin/bash
~~~

3\. Source ROS workspace

~~~bash
source devel/setup.bash
~~~

4\. Start the Roscore, set the ROS parameters, start the server endpoint, start the Mover Service and Pose Estimation nodes and launch MoveIt:

~~~bash
roslaunch ur3_moveit pose_est.launch 
~~~

This launch file loads all ROS related files, starts ROS nodes required for trajectory planning for the UR3 robot. 

## References

For more information, reference the following tutorial series because in the description,
they provide the link to download their assets including Dockerfiles to build
their docker image.

Later we will have our own docker image.

- Robotics Development: ROS, Unity3D, DL & DevOps Tools (Tutorial Series): https://youtube.com/playlist?list=PLB8VXMjsTRoue4aodor1lDyZhFTKEP58i&si=e_JukXpAezD40sQt