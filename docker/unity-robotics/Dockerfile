FROM ros:noetic-ros-base

RUN sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654

RUN sudo apt-get update && sudo apt-get install -y vim iputils-ping net-tools python3-pip ros-noetic-robot-state-publisher ros-noetic-moveit ros-noetic-rosbridge-suite ros-noetic-joy ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-tf* ros-noetic-gazebo-ros-pkgs ros-noetic-joint-state-publisher ros-noetic-soem ros-noetic-ros-canopen dos2unix git

RUN sudo -H pip3 --no-cache-dir install rospkg numpy jsonpickle scipy easydict torch==1.7.1+cu101 torchvision==0.8.2+cu101 torchaudio==0.7.2 -f https://download.pytorch.org/whl/torch_stable.html

ENV ROS_WORKSPACE=/catkin_ws

WORKDIR $ROS_WORKSPACE

RUN mkdir -p /opt/guz/ext-ros/comp
RUN chmod -R 777 /opt/guz/ext-ros

COPY ./scripts/comp/10-clone-moveit_msgs.sh /opt/guz/ext-ros/comp
RUN chmod +x /opt/guz/ext-ros/comp/10-clone-moveit_msgs.sh
RUN bash /opt/guz/ext-ros/comp/10-clone-moveit_msgs.sh

COPY ./scripts/comp/20-clone-robotiq.sh /opt/guz/ext-ros/comp
RUN chmod +x /opt/guz/ext-ros/comp/20-clone-robotiq.sh
RUN bash /opt/guz/ext-ros/comp/20-clone-robotiq.sh

# For now need to use the particular ROS TCP Endpoint from Unity's Robotics-Object-Pose-Estimation
# COPY ./scripts/comp/30-clone-ros-tcp-endpoint.sh /opt/guz/ext-ros/comp
# RUN chmod +x /opt/guz/ext-ros/comp/30-clone-ros-tcp-endpoint.sh
# RUN bash /opt/guz/ext-ros/comp/30-clone-ros-tcp-endpoint.sh

COPY ./scripts/comp/40-clone-universal_robot.sh /opt/guz/ext-ros/comp
RUN chmod +x /opt/guz/ext-ros/comp/40-clone-universal_robot.sh
RUN bash /opt/guz/ext-ros/comp/40-clone-universal_robot.sh

COPY ./scripts/comp/50-clone-unity_ur3_moveit.sh /opt/guz/ext-ros/comp
RUN chmod +x /opt/guz/ext-ros/comp/50-clone-unity_ur3_moveit.sh
RUN bash /opt/guz/ext-ros/comp/50-clone-unity_ur3_moveit.sh

COPY ./ROS/src/ur3_moveit/models $ROS_WORKSPACE/src/ur3_moveit/models

COPY set-up-workspace /setup.sh
COPY tutorial /tutorial

RUN /bin/bash -c "find $ROS_WORKSPACE -type f -print0 | xargs -0 dos2unix"

RUN dos2unix /tutorial
RUN dos2unix /setup.sh
RUN chmod +x /setup.sh
RUN /setup.sh

# # # pre-load model
RUN python3 -c 'import src.ur3_moveit.src.ur3_moveit.setup_and_run_model as model; model.preload()'

ENTRYPOINT ["bash", "/tutorial"]
