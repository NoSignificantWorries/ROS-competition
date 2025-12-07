FROM osrf/ros:humble-desktop-full

ARG USERNAME=ubuntu

# 1. Сначала создать пользователя и его домашнюю директорию
RUN useradd -m -s /bin/bash $USERNAME && \
  echo "$USERNAME:$USERNAME" | chpasswd && \
  adduser $USERNAME sudo

# 2. Установить пароль для пользователя (опционально)
RUN echo "ubuntu:ubuntu" | chpasswd

RUN apt-get update -y && apt-get upgrade -y
RUN apt-get install -y vim python3-colcon-ed tree tmux neovim curl wget gnupg lsb-release
RUN apt-get install -y ros-humble-rviz2 ros-humble-turtle-tf2-py ros-humble-tf2-ros ros-humble-tf2-tools ros-humble-turtlesim ros-humble-teleop-twist-keyboard ros-humble-rqt-robot-steering ros-humble-ros-gz-sim ros-humble-ros-gz-interfaces ros-humble-ros-gz-bridge ros-humble-rviz-imu-plugin

# 3. ОБНОВЛЕНИЕ: Установка правильного Gazebo для Humble (Fortress)
# УДАЛИТЬ старые строки с установкой gazebo-stable.list
RUN apt-get update && apt-get install -y ros-humble-ros-gz

SHELL ["/usr/bin/bash", "-c"]

# 4. Теперь домашняя директория существует, можно настраивать .bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc && \
  echo "export LIBGL_ALWAYS_SOFTWARE=1" >> /home/$USERNAME/.bashrc && \
  echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> /home/$USERNAME/.bashrc && \
  echo "export _colcon_cd_root=/opt/ros/humble/" >> /home/$USERNAME/.bashrc

WORKDIR /home/ubuntu/workbench

# 5. Переключиться на созданного пользователя
USER ubuntu

# FROM osrf/ros:humble-desktop-full
#
# ARG USERNAME=ubuntu
#
# RUN apt-get update -y && apt-get upgrade -y
# RUN apt-get install -y vim python3-colcon-ed tree tmux neovim curl wget gnupg lsb-release
# RUN apt-get install -y ros-humble-rviz2 ros-humble-turtle-tf2-py ros-humble-tf2-ros ros-humble-tf2-tools ros-humble-turtlesim ros-humble-teleop-twist-keyboard ros-humble-rqt-robot-steering ros-humble-ros-gz-sim ros-humble-ros-gz-interfaces ros-humble-ros-gz-bridge ros-humble-rviz-imu-plugin

# RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
# RUN wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -
#
# RUN apt-get update
# RUN apt-get install -y libgz-sensors10-lidar libgz-sensors10-gpu-lidar libgz-sensors10-imu

# SHELL ["/usr/bin/bash", "-c"]
#
# RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc && \
#   echo "export LIBGL_ALWAYS_SOFTWARE=1" >> /home/$USERNAME/.bashrc && \
#   echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> /home/$USERNAME/.bashrc && \
#   echo "export _colcon_cd_root=/opt/ros/humble/" >> /home/$USERNAME/.bashrc
#
# WORKDIR /home/ubuntu/workbench
#
# USER ubuntu
