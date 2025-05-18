# base image
FROM ubuntu:22.04

# setting environmental variables
ENV DEBIAN_FRONTEND noninteractive
ARG MAKEFLAGS
ARG PARALLELWORKERS
ARG DISPLAY
ARG WAYLAND_DISPLAY
ARG XDG_RUNTIME_DIR
ARG PULSE_SERVER
ENV MAKEFLAGS $MAKEFLAGS
ENV PARALLELWORKERS $PARALLELWORKERS
ENV DISPLAY $DISPLAY
ENV WAYLAND_DISPLAY $WAYLAND_DISPLAY
ENV XDG_RUNTIME_DIR $XDG_RUNTIME_DIR
ENV PULSE_SERVER $PULSE_SERVER

# setting time zone
RUN apt update && apt install -y --no-install-recommends tzdata
ENV TZ=Asia/Tokyo

RUN apt update -y && apt upgrade -y && apt install -y --no-install-recommends \
    software-properties-common \
    texlive-fonts-recommended \
    texlive-latex-extra \
    libxkbcommon-x11-0 \
    gnome-screenshot \
    libcanberra-gtk* \
    build-essential \
    libgl1-mesa-dev \
    python3-numpy \
    iputils-ping \
    lsb-release \
    libudev-dev \
    imagemagick \
    python3-dev \
    python3-pip \
    python3-tk \
    mesa-utils \
    net-tools \
    aptitude \
    xorg-dev \
    x11-apps \
    libfuse2 \
    cm-super \
    texlive \
    libxcb* \
    dvipng \
    unzip \
    gnupg \
    byobu \
    cmake \
    curl \
    wget \
    dbus \
    git \
    vim \
    zip \
    && rm -rf /var/lib/apt/lists/*

# adding the ROS2 GPG key with apt and add the repository to your sources list
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN /bin/bash -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null'

# updating mesa (fix the issue of displaying black world in Rviz2 and Gazebo)
# https://github.com/ros2/rviz/issues/948
RUN add-apt-repository ppa:kisak/kisak-mesa -y && apt update -y

# installing ROS2 packages and colcon build system with mixin
# changing the middleware (CycloneDDS recommended for MoveIt2)
RUN apt install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-vcstool \
    python3-rosdep \
    ros-humble-desktop \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-ament-cmake \
    ros-humble-warehouse-* \
    ros-humble-gazebo-* \
    ros-humble-ros-gz* \
    ros-humble-moveit* \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-ur \
    ros-dev-tools \
    gazebo \
    && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc

RUN /bin/bash -c "colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && colcon mixin update default"

RUN mkdir -p /tmp/
RUN mkdir -p /root/ros2_ws/src
WORKDIR /root/ros2_ws
RUN	/bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"
RUN	echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
	echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc && \
	echo "export ROS_PACKAGE_PATH=\${ROS_PACKAGE_PATH}:/root/ros2_ws" >> ~/.bashrc && \
	echo "export ROS_WORKSPACE=/root/ros2_ws" >> ~/.bashrc && \
    echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc && \
    echo "export _colcon_cd_root=/opt/ros/humble/" >> ~/.bashrc

# building ros2 packages
WORKDIR /root/ros2_ws/src
RUN git clone https://github.com/ros-simulation/gazebo_ros2_control.git -b humble --recursive && \
    cd gazebo_ros2_control && \
    git reset --hard b49ed0b
RUN git clone https://github.com/moveit/moveit2.git -b humble --recursive && \
    cd moveit2 && \
    git reset --hard a60ed9f && \
    cd .. && \
    vcs import < moveit2/moveit2.repos
WORKDIR /root/ros2_ws/
RUN /bin/bash -c "rosdep init && rosdep update && rosdep install -r --from-paths . --ignore-src --rosdistro humble -y"

# building ros2 packages
WORKDIR /root/ros2_ws/
RUN git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git src/Universal_Robots_ROS2_Driver -b humble && \
    cd src/Universal_Robots_ROS2_Driver && git reset --hard 5dfed5d && \
    cd ../../ && vcs import src --skip-existing --input src/Universal_Robots_ROS2_Driver/Universal_Robots_ROS2_Driver-not-released.humble.repos
RUN /bin/bash -c "rosdep update && rosdep install -r --from-paths . --ignore-src --rosdistro humble -y" 

# Generates bash commands
WORKDIR /root/ros2_ws/src/

# Moving the robot with a python script, while checking it with rviz
RUN echo "ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=127.0.0.1 use_fake_hardware:=true initial_joint_controller:=scaled_joint_trajectory_controller launch_rviz:=true" >> ~/.bash_history
RUN echo "ros2 run ur3e_tutorials hello_joint_trajectory_controller"
# or
RUN echo "ros2 launch ur3e_tutorials hello_joint_trajectory_controller.launch.py" >> ~/.bash_history

# MoveIt GUI
RUN echo "ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e launch_rviz:=true" >> ~/.bash_history

# Visualizing the robot model
RUN echo "ros2 launch ur_description view_ur.launch.py ur_type:=ur3e" >> ~/.bash_history
