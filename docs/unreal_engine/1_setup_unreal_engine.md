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

### Launch Unreal Engine using Prebuilt Binaries Release

1\. If we download the prebuilt binaries for Linux Unreal Engine, then we create a `bin` folder in `~/` folder, move the Unreal Engine prebuilt binaries folder to bin and then run Unreal Engine:

~~~bash
# Refer to "Unreal Engine for Linux Downloads"
mkdir ~/bin/
cd ~/bin/Linux_Unreal_Engine_5.3.2/Engine/Binaries/Linux
./UnrealEditor
~~~

### Build Unreal Engine from Source & Launch Unreal

1\. If we clone Unreal Engine 5 C++ from source, we can build it and then run it

~~~bash
# Refer to "Accessing Unreal Engine source code on GitHub"

# Prereq: make sure your github is linked to your epic games account
# and got access to the Unreal Engine C++ source code
git clone git@github.com:EpicGames/UnrealEngine.git -b 5.3.2-release --recursive

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


## Appendix: DeepMind MuJoCo Physics Simulator

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
