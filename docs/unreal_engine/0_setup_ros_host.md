# Install ROS Jazzy Host Environment

## Outline

- 1\. System Setup
- 2\. Install ROS2
- 3\. Setup ROS2 Environment
- 4\. Install RosDep
- 5\. Install Colcon
- 6\. Install ROS2 Bridge Suite
- Appendix A: Add urobosim_msgs package to ROS2 Workspace

ROS2 Jazzy Installation Reference (Ubuntu 24.04): https://docs.ros.org/en/jazzy/Installation.html

ROS2 Jazzy Installation Deb Packages Reference (Ubuntu 24.04): https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

ROS2 Bridge Suite (Jazzy) github repo: https://github.com/RobotWebTools/rosbridge_suite

ROS Bridge Suite package summary: https://wiki.ros.org/rosbridge_suite

ROS2 URoboSim_Msgs: https://github.com/james94/urobosim_msgs

Managing Dependencies with RosDep (ROS2 Jazzy): https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Rosdep.html

Install Colcon (ROS2 Jazzy): https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html#install-colcon

## 1. System Setup

### 1.1 Set Locale for UTF-8

~~~bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
~~~

### 1.2 Enable Required ROS2 Apt Repositories

Ensure the Ubuntu Universe repository is enabled:

~~~bash
sudo apt -y install software-properties-common
sudo add-apt-repository universe
~~~

Add the ROS2 GPG key with apt:

~~~bash
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
~~~


Add the repository to your sources list:

~~~bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
~~~

## 2. Install ROS2

~~~bash
# Update apt repo caches
sudo apt update

# ROS2 packages are built on frequently updated Ubuntu systems
  # Ensure your system is up-to-date before installing new packages
sudo apt upgrade

# sudo apt install ros-jazzy-desktop

# ROS2 Base: communication libraries, message packages, command line tools (no gui tools)
sudo apt -y install ros-jazzy-ros-base
~~~

## 3. Setup ROS2 Environment

Set up ROS2 environment:

~~~bash
# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setup.sh, setup.zsh
source /opt/ros/jazzy/setup.bash
~~~

## 4. Install RosDep

Since we will use rosdep with ROS2, it is packaged with ROS distribution, we can install rosdep with the command:

~~~bash
sudo apt-get -y install python3-rosdep
~~~

## 5. Install Colcon

Install ROS2 colcon, so we can build ROS2 code:

~~~bash
sudo apt -y install python3-colcon-common-extensions
~~~

## 6. Install ROS2 Bridge Suite

Install ROS2 Bridge Suite:

~~~bash
sudo apt-get -y install ros-jazzy-rosbridge-server
~~~

## Appendix A: Add urobosim_msgs package to ROS2 Workspace

### 1. Setup ROS2 Workspace

1\. Create a Workspace:

~~~bash
mkdir -p $HOME/Robot-Arm-AI-Simulation/ros2_ws/src
cd $HOME/Robot-Arm-AI-Simulation/ros2_ws
~~~

2\. Source ROS2 Environment

~~~bash
source /opt/ros/jazzy/setup.bash
~~~

### 2. Clone urobosim_msgs Repository

3\. Navigate to src directory:

~~~bash
cd $HOME/Robot-Arm-AI-Simulation/ros2_ws/src
~~~

4\. Add urobosim_msgs repository as submodule:

~~~bash
git submodule add git@github.com:james94/urobosim_msgs.git
~~~

### 3. Resolve Dependencies

5\. Install Dependencies

~~~bash
cd $HOME/Robot-Arm-AI-Simulation/ros2_ws

# If first time running rosdep, it must be initialized
sudo rosdep init
rosdep update

# install dependencies, we run this over our workspace ros2_ws/
  # that has urobosim_msgs package, so rosdep will install our package's depencies
rosdep install --from-paths src --ignore-src -r -y
~~~

Heres the output of the rosdept install ... command:

<!-- ubuntu@server:~/src/Robot-Arm-AI-Simulation/ros2_ws$ rosdep install --from-paths src --ignore-src -r -y
ERROR: the following packages/stacks could not have their rosdep keys resolved
to system dependencies:
urobosim_msgs: Cannot locate rosdep definition for [catkin]
Continuing to install resolvable dependencies... -->

~~~bash
#All required rosdeps installed successfully
~~~

### 4. Build the Workspace with Colcon

We use colcon to build ROS2 workspace, which will compile all the packages,
including `urobosim_msgs`:

~~~bash
colcon build
~~~

- NOTE: if you run into a build issue with **CMAKE_PREFIX_PATH**, try: `unset CMAKE_PREFIX_PATH` to remove the non-existent path
- Then retry sourcing the ROS2 Jazzy setup and then rerun colcon build

For more info on migrating from ROS kinetic to ROS2 jazzy, refer to perplexity AI: https://www.perplexity.ai/search/you-are-a-software-engineer-wi-GsVq_LksTjejC2PcsyuO3g

### 5. Source the Overlay

After building, source the overlay setup file to overlay new packages, including 
`urobosim_msgs` on top of the existing ROS2 installation:

~~~bash
source $HOME/Robot-Arm-AI-Simulation/ros2_ws/install/setup.bash
~~~

Now we are ready to launch [Unreal Engine for our URX Robot Arm project](./docs/unreal_engine/1_setup_unreal_engine.md)
