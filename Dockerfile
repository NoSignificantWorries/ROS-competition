FROM osrf/ros:jazzy-desktop-full-noble

ARG USERNAME=ubuntu

RUN apt-get update -y && apt-get upgrade -y
RUN apt-get install -y vim python3-colcon-ed tree tmux neovim curl wget gnupg lsb-release
RUN apt-get install -y ros-jazzy-rviz2 ros-jazzy-turtle-tf2-py ros-jazzy-tf2-ros ros-jazzy-tf2-tools ros-jazzy-turtlesim ros-jazzy-teleop-twist-keyboard ros-jazzy-rqt-robot-steering ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-interfaces ros-jazzy-ros-gz-bridge ros-jazzy-rviz-imu-plugin

RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -

RUN apt-get update
RUN apt-get install -y libgz-sensors10-lidar libgz-sensors10-gpu-lidar libgz-sensors10-imu

SHELL ["/usr/bin/bash", "-c"]

RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/$USERNAME/.bashrc && \
  echo "export LIBGL_ALWAYS_SOFTWARE=1" >> /home/$USERNAME/.bashrc && \
  echo "export GZ_SIM_RESOURCE_PATH=/home/ubuntu:/root:\$GZ_SIM_RESOURCE_PATH" >> /root/.bashrc && \
  echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> /home/$USERNAME/.bashrc && \
  echo "export _colcon_cd_root=/opt/ros/jazzy/" >> /home/$USERNAME/.bashrc

WORKDIR /home/ubuntu/workbench

USER ubuntu

# FROM osrf/ros:humble-desktop-full
#
# ARG USERNAME=ubuntu
#
# RUN useradd -m -s /bin/bash $USERNAME && \
#   echo "$USERNAME:$USERNAME" | chpasswd && \
#   adduser $USERNAME sudo
#
# RUN echo "ubuntu:ubuntu" | chpasswd
#
# RUN apt-get update -y && apt-get upgrade -y
# RUN apt-get install -y vim python3-colcon-ed tree tmux neovim curl wget gnupg lsb-release
# RUN apt-get install -y ros-humble-rviz2 ros-humble-turtle-tf2-py ros-humble-tf2-ros ros-humble-tf2-tools ros-humble-turtlesim ros-humble-teleop-twist-keyboard ros-humble-rqt-robot-steering ros-humble-ros-gz-sim ros-humble-ros-gz-interfaces ros-humble-ros-gz-bridge ros-humble-rviz-imu-plugin
#
# RUN apt-get update && apt-get install -y ros-humble-ros-gz
#
# SHELL ["/usr/bin/bash", "-c"]
#
# RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
#   echo "export LIBGL_ALWAYS_SOFTWARE=1" >> /root/.bashrc && \
#   # echo "export GZ_SIM_RESOURCE_PATH=/root/robot_bringup/worlds:/root:\$GZ_SIM_RESOURCE_PATH" >> /root/.bashrc && \
#   echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> /root/.bashrc && \
#   echo "export _colcon_cd_root=/opt/ros/humble/" >> /root/.bashrc
#
# RUN echo "export GZ_SIM_RESOURCE_PATH=/root/robot_bringup/worlds:/root:\$GZ_SIM_RESOURCE_PATH" >> /home/ubuntu/.bashrc
#
# WORKDIR /root
#
# USER ubuntu
#
