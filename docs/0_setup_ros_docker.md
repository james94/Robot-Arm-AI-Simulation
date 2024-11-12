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

## Troubleshooting

### Forgot Git LFS Pull UR3_MoveIt DL Model

If you were trying to run Unity's **ur3_moveit pose_est.launch** file and after you ran the Unity Robotic Object Pose Estimation simulation, from the ur3_moveit pose_est.launch script, you obtained the following error, how can you resolve it?

~~~log
[ERROR] [1731403524.024244]: Error processing request: invalid load key, 'v'.
['Traceback (most recent call last):\n', '  File "/opt/ros/noetic/lib/python3/dist-packages/rospy/impl/tcpros_service.py", line 633, in _handle_request\n    response = convert_return_to_response(self.handler(request), self.response_class)\n', '  File "/catkin_ws/src/ur3_moveit/scripts/pose_estimation_script.py", line 96, in pose_estimation_main\n    est_position, est_rotation = _run_model(image_path)\n', '  File "/catkin_ws/src/ur3_moveit/scripts/pose_estimation_script.py", line 52, in _run_model\n    output = run_model_main(image_path, MODEL_PATH)\n', '  File "/catkin_ws/src/ur3_moveit/src/ur3_moveit/setup_and_run_model.py", line 130, in run_model_main\n    checkpoint = torch.load(model_file_name, map_location=device)\n', '  File "/usr/local/lib/python3.8/dist-packages/torch/serialization.py", line 595, in load\n    return _legacy_load(opened_file, map_location, pickle_module, **pickle_load_args)\n', '  File "/usr/local/lib/python3.8/dist-packages/torch/serialization.py", line 764, in _legacy_load\n    magic_number = pickle_module.load(f, **pickle_load_args)\n', "_pickle.UnpicklingError: invalid load key, 'v'.\n"]
~~~

- Potential Solution: double check the model file is the correct size. For example, on Dell's Ubuntu, we see a smaller model size after cloning our Robot Arm AI Simulation repo:

~~~log
ubuntu@ubuntu-zm26 ~/src/james/Robot-Arm-AI-Simulation/docker/unity-robotics/ROS/src/ur3_moveit/models (main)$ ls -ltr
total 4
-rw-rw-r-- 1 ubuntu ubuntu 134 Nov 10 21:53 UR3_single_cube_model.tar
~~~

For example, I forgot to git lfs pull files, and this model actually depends on LFS. The actual model size as we had on Bizon Ubuntu workstation should be closer to:

~~~log
bizon@dl ~/src/Robot-Arm-AI-Simulation/docker/unity-robotics/ROS/src/ur3_moveit/models (main)$ ls -ltr
total 107812
-rw-rw-r-- 1 bizon bizon 110391301 Nov  6 22:48 UR3_single_cube_model.tar
~~~

Once you do a git lfs, you should get the model's expected size and ideally, you shouldn't run into the error we saw above.

~~~bash
cd Robot-Arm-AI-Simulation/

# Ensure Git LFS is installed on your system
sudo apt -y install git-lfs

# Initialize Git LFS for your system
git lfs install

# Ensure all LFS files are downloaded successfully (needed for .tar, etc)
git lfs pull
~~~

After doing git lfs pull, we should see our Dell's Ubuntu Robot-Arm-AI-Simulation repo's model size is about the same as the model from our Bizon Ubuntu workstation.

~~~log
ubuntu@ubuntu-zm26 ~/src/james/Robot-Arm-AI-Simulation/docker/unity-robotics/ROS/src/ur3_moveit/models (main)$ ls -ltr
total 107812
-rw-rw-r-- 1 ubuntu ubuntu 110391301 Nov 12 01:48 UR3_single_cube_model.tar
~~~

### Adjust UR3_MoveIt Planning Process Timing

I realized the error above related to the particular pytorch version was because I launched the docker container to use my native host computers gpus. So, I relaunched docker container, so it doesnt use my native host computer's gpus and instead just use docker container's pytorch version. When I reran ur3_moveit pose_est.launch, I get the following error, how can we resolve it?

- **NOTE:** I encountered this error when I switched from Bizon AI workstation to Dell XPS. The Dell XPS has less Nvidia 3050 Ti GPU memory compared to Bizon AI workstation having Nvidia 4090 GPU memory.

~~~log
You can start planning now!

ROS-Unity Handshake received, will connect to 127.0.0.1:5005
Started estimation pipeline
Predicting from screenshot /catkin_ws/src/ur3_moveit/images/Input1.png
/usr/local/lib/python3.8/dist-packages/torch/cuda/__init__.py:52: UserWarning: CUDA initialization: Found no NVIDIA driver on your system. Please check that you have an NVIDIA GPU and installed a driver from http://www.nvidia.com/Download/index.aspx (Triggered internally at  /pytorch/c10/cuda/CUDAFunctions.cpp:100.)
  return torch._C._cuda_getDeviceCount() > 0
[ INFO] [1731405694.834266286]: Loading robot model 'ur3_with_gripper'...
Finished estimation pipeline

[ INFO] [1731405696.005793387]: Ready to take commands for planning group arm.
[ WARN] [1731405698.011432770]: Empty quaternion found in pose message. Setting to neutral orientation.
[ INFO] [1731405698.069275123]: Planning request received for MoveGroup action. Forwarding to planning pipeline.
[ INFO] [1731405698.070862634]: Planner configuration 'arm' will use planner 'geometric::RRTConnect'. Additional configuration parameters will be set when the planner is constructed.
[ INFO] [1731405698.071137626]: arm/arm: Starting planning with 1 states already in datastructure
[ERROR] [1731405703.079280046]: arm/arm: Unable to sample any valid states for goal tree
[ INFO] [1731405703.079334041]: arm/arm: Created 1 states (1 start + 0 goal)
[ INFO] [1731405703.079349800]: No solution found after 5.008314 seconds
[ WARN] [1731405703.079390129]: Timed out
[ INFO] [1731405703.094615704]: Unable to solve the planning problem
[ WARN] [1731405703.094969720]: Fail: ABORTED: TIMED_OUT
[ERROR] [1731405703.095448]: Error processing request: list index out of range
['Traceback (most recent call last):\n', '  File "/opt/ros/noetic/lib/python3/dist-packages/rospy/impl/tcpros_service.py", line 633, in _handle_request\n    response = convert_return_to_response(self.handler(request), self.response_class)\n', '  File "/catkin_ws/src/ur3_moveit/scripts/mover.py", line 78, in plan_pick_and_place\n    previous_ending_joint_angles = pre_grasp_pose.joint_trajectory.points[-1].positions\n', 'IndexError: list index out of range\n']
Service Exception raised. Attempt: 1
~~~

Heres a breakdown of the issues and potential solutions:

**Issue 1\. Planning Failure:**

The planner (RRTConnect) was unable to find a valid solution within the 5-second timeout. This could be due to several reasons:

- Complex motion planning problem
- Unreachable target pose
- Insufficient planning time

**Issue 2\. IndexError:**

The error "list index out of range" occurs because the planner couldn't generate a valid trajectory, so there are no points in the joint_trajectory.

After considering the issues above, in my case with when switching from my powerful Bizon AI workstation to Dell XPS laptop, I found this solution to allow **ur3_moveit pose_est.launch** to succeed:

**Solution 1: Increase planning time:**

Modify the plan_pick_and_place function in mover.py to allow more time for planning. Increase the planning_time parameter:

~~~python
plan = move_group.plan()
success = plan[0]
~~~

Change to:

~~~python
move_group.set_planning_time(10)  # Increase to 10 seconds or more
plan = move_group.plan()
success = plan[0]
~~~

**NOTE:** our docker image comes prebuilt with the ur3_moveit's **mover.py** modified to account for needing more time for planning.

Also you can reference more information on the solution 1 and other potential solutions from perplexity AI: https://www.perplexity.ai/search/you-are-a-software-engineer-ai-9MVpi3.qSbG8fkNKG_SRNg#2
