# Setup Unreal Engine Dev Environment

## Outline

- [Unreal Engine 5](#unreal-engine-5)
- [Appendix: DeepMind's MuJoCo Physics Engine](#deepmind-mujoco-physics-simulator)

## Unreal Engine 5

- [Unreal Engine for Linux Downloads](https://www.unrealengine.com/en-US/linux)
- [UE5 Dev Requirements](https://docs.unrealengine.com/5.0/en-USlinux-development-requirements-for-unreal-engine/)
- [UE5 Docker Containers](https://docs.unrealengine.com/5.0/en-USquick-start-guide-for-using-container-images-in-unreal-engine/)
- [Accessing Unreal Engine source code on GitHub](https://www.unrealengine.com/en-US/ue-on-github)
    - Full access to the complete C++ source code, so you can study, customize, extend, debug theentire Unreal Engine and complete project without obstruction
- [Build scripts for native Linux build](https://github.com/EpicGames/UnrealEngine/blob/releaseEngine/Build/BatchFiles/Linux/README.md)

## Dependencies

Unreal Engine Plugins we use include the following:

- UUtils: https://github.com/james94/UUtils
    - 20 commits ahead in URoboSim/UUtils to RobCoG-IAI/UUtils: https://github.com/robcog-iai/UUtils/compare/master...urobosim:UUtils:master

- UROSBridge: https://github.com/james94/UROSBridge
    - 9 commits ahead in URoboSim/UROSBridge to RobCoG-IAI/UROSBridge: https://github.com/robcog-iai/UROSBridge/compare/master...urobosim:UROSBridge:master


- UROSWorldControl: https://github.com/james94/UROSWorldControl
    - 20 commits ahead in URoboSim/UROSWorldControl to RobCoG-IAI/UROSWorldControl: https://github.com/robcog-iai/UROSWorldControl/compare/master...urobosim:UROSWorldControl:master

- URoboVision: https://github.com/james94/URoboVision
    - 12 commits ahead in URoboSim/URoboVision to RobCoG-IAI/URoboVision: https://github.com/robcog-iai/URoboVision/compare/master...urobosim:URoboVision:master

- UTFPublisher: https://github.com/robcog-iai/UTFPublisher

- URoboSim: https://github.com/james94/URoboSim

### Launch Unreal Engine using Prebuilt Binaries Release

1\. If we download the prebuilt binaries for Linux Unreal Engine, then we create a `bin` folder in `~/` folder, move the Unreal Engine prebuilt binaries folder to bin and then run Unreal Engine:

~~~bash
# Refer to "Unreal Engine for Linux Downloads"
mkdir ~/bin/
cd ~/bin/Linux_Unreal_Engine_5.5.0/Engine/Binaries/Linux
./UnrealEditor
~~~

### Build Unreal Engine from Source & Launch Unreal

1\. If we clone Unreal Engine 5 C++ from source, we can build it and then run it

~~~bash
# Refer to "Accessing Unreal Engine source code on GitHub"

# Prereq: make sure your github is linked to your epic games account
# and got access to the Unreal Engine C++ source code
git clone git@github.com:EpicGames/UnrealEngine.git -b 5.5.0-release --recursive

cd UnrealEngine
./Setup.sh

# Generate CMakefiles and makefiles
./GenerateProjectFiles.sh

# Build the Unreal Editor from scratch
make

# Or rebuild the editor from scratch
make UnrealEditor ARGS="-clean" && make UnrealEditor

# Run it
cd Engine/Binaries/Linux/
./UnrealEditor
~~~

## Build Unreal Engine C++ Plugin using Automation Tool

~~~bash
cd ~/bin/Linux_Unreal_Engine_5.5.0/Engine/Build/BatchFiles

./RunUAT.sh BuildPlugin -plugin="/home/ubuntu/src/james/Robot-Arm-AI-Simulation/unreal/ue_5_5_0/UR3GripObjPosePred/Plugins/UUtils/UUtils.uplugin" -package="/home/ubuntu/src/james/Robot-Arm-AI-Simulation/unreal/ue_5_5_0/UR3GripObjPosePred/Plugins/UUtils//UUtils_UE550" -clean

# Compile UROSBridge C++ Plugin from Source using UE's Automation Tool
./RunUAT.sh BuildPlugin -plugin="/home/ubuntu/src/james/Robot-Arm-AI-Simulation/unreal/ue_5_5_0/UR3GripObjPosePred/Plugins/UROSBridge/UROSBridge.uplugin" -package="/home/ubuntu/src/james/Robot-Arm-AI-Simulation/unreal/ue_5_5_0/UR3GripObjPosePred/Plugins/UROSBridge/UROSBridge_UE550" -clean

# Compile UROSWorldControl C++ Plugin from Source using UE's Editor Build Tool while compiling our main UR3GripObjPosePred project
/home/ubuntu/bin/Linux_Unreal_Engine_5.5.0/Engine/Build/BatchFiles/Linux/Build.sh Development Linux -Project="/home/ubuntu/src/james/Robot-Arm-AI-Simulation/unreal/ue_5_5_0/UR3GripObjPosePred/UR3GripObjPosePred.uproject" -TargetType=Editor -Progress
~~~


## Build Unreal Engine C++ Plugin using Unreal Editor Build Tool

~~~bash
# Compile UROSWorldControl C++ Plugin from Source using UE's Editor Build Tool while compiling our main UR3GripObjPosePred project
    # NOTE (JG): I found using the UE Automation Tool for building plugins, it couldn't find the dependency plugins in UROSWorldControl.uplugin file, which are
    # UROSBridge and UUTils; However, when we build our main UR3GripObjPosePred project along with the Plugins, its able to find those dependency plugins
# Next we added URoboVision plugin, so now we're also compiling that new plugin along with the other plugins too
/home/ubuntu/bin/Linux_Unreal_Engine_5.5.0/Engine/Build/BatchFiles/Linux/Build.sh Development Linux -Project="/home/ubuntu/src/james/Robot-Arm-AI-Simulation/unreal/ue_5_5_0/UR3GripObjPosePred/UR3GripObjPosePred.uproject" -TargetType=Editor -Progress
~~~


## Importing New Robot Model

From within ROS2 noetic docker container, convert UR3 URDF to SDF for importing into Unreal Engine:

~~~bash
# testing ROS2 noetic docker container related to unreal engine
docker run -it --rm -p 10000:10000 -p 5005:5005 \
-v /home/ubuntu/src/james/Robot-Arm-AI-Simulation/unreal/ue_5_5_0/ur3_moveit:/src/unreal/ue_5_5_0/ur3_moveit \
unity-robotics:pose-estimation /bin/bash

source devel/setup.bash

pushd /src/unreal/ue_5_5_0/ur3_moveit

# Convert URDF to SDF for UR3 robot arm with gripper
gz sdf -p URDF/ur3_with_gripper.urdf  > SDF/ur3_with_gripper.sdf

# replace the single quote (') with double quote (")
sed -i "s/'/\"/g" SDF/ur3_with_gripper.sdf

popd
~~~

Install 

- Reference URoboSim github page: https://github.com/james94/URoboSim/tree/master

## Appendix A: DeepMind MuJoCo Physics Simulator

The MuJoCo Physics Engine Simulator comes with plugin support for Unity 3D integration. **However, there is still work that needs to be done for plugin support for Unreal Engine 5 integration.**

DeepMind's "MuJoCo Menagerie" Physics Engine Simulator that contains over 8 simulated digital twins: robotic's hand, gripper, legs, robot, dog, robotic arm, drone and realsense camera:

- [Universal Robots UR5e — MuJoCo Menagerie](https://www.youtube.com/watch?v=gAqwNeY0juo)

- [Kevin Zakka's MuJoCo Physics Simulation Demo Videos](https://www.youtube.com/@kevin_zakka/videos)

- [MuJoCo Learning Playlist 29 videos](https://www.youtube.com/playlist?list=PLzWiCM7Cn8WYCrvMYGszduvFhkw-RvjAl)

- [MuJoCo200 (based on DeepMind's MuJoCo) Tutorials](https://www.youtube.com/playlist?list=PLc7bpbeTIk758Ad3fkSywdxHWpBh9PM0G)

- [MuJoCo Python Tutorials](https://www.youtube.com/playlist?list=PLc7bpbeTIk75dgBVd07z6_uKN1KQkwFRK)

### MuJoCo (Multi-Joint Dynamics with Contact) Physics Engine

- [DeepMind MuJoCo Physics Engine GitHub Link](https://github.com/google-deepmind/mujoco)
    - Multi-Joint dynamics with Contact. A general purpose physics simulator.

### MuJoCo Menagerie Sim-To-Real Collection Models

- [DeepMind MuJoCo Menagerie GitHub Link](https://github.com/google-deepmind/mujoco_menagerie)
    - A collection of high-quality models for the MuJoCo physics engine, curated by Google DeepMind.

## Appendix B: Unreal Engine

- Building Unreal Engine Plugins: https://dev.epicgames.com/community/learning/tutorials/qz93/unreal-engine-building-plugins
