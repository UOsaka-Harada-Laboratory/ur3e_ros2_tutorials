# ur3e_ros2_tutorials

[![support level: community](https://img.shields.io/badge/support%20level-community-lightgray.svg)](http://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)
[![license: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
![repo size](https://img.shields.io/github/repo-size/UOsaka-Harada-Laboratory/ur3e_tutorials)

- ROS2 packages for Universal Robots UR3e tutorial.
    - [ur3e_tutorials_py](/ros2_ws/src/ur3e_tutorials_py): A tutorial package to execute simple demonstrations with Python.
    - [ur3e_tutorials_cpp](/ros2_ws/src/ur3e_tutorials_cpp): A tutorial package to execute simple demonstrations with C++.
- Docker for simulation and control environments for Universal Robots UR3e.

## Dependencies

### Docker build environment

- [Ubuntu 22.04 PC](https://ubuntu.com/certified/laptops?q=&limit=20&vendor=Dell&vendor=Lenovo&vendor=HP&release=22.04+LTS)
    - Docker 27.4.1
    - Docker Compose 2.32.1

### UR3e with a robotiq gripper

- [Ubuntu 22.04 PC](https://ubuntu.com/certified/laptops?q=&limit=20&vendor=Dell&vendor=Lenovo&vendor=HP&release=22.04+LTS)
    - [ROS Melodic](https://wiki.ros.org/melodic/Installation/Ubuntu)
    - [UniversalRobots/Universal_Robots_ROS_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)
    - [fmauch/universal_robot](https://github.com/fmauch/universal_robot.git)
    - [Byobu](https://www.byobu.org/)
- [UniversalRobots UR3e](https://www.universal-robots.com/products/ur3-robot/) 
- [Robotiq Hand-E](https://robotiq.com/products/adaptive-grippers#Hand-E)

## Installation

### Teach pendant

1. Install URCap on a e-series robot by following [here](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_client_library/doc/setup/robot_setup.html).
    - Don't forget the last step of starting the External Control program on the teach pendant.
    - After bringup the system, start the External Control program to establish the connection.  

2. (option to use a gripper) Install [tool communication](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_robot_driver/doc/setup_tool_communication.html#setup-tool-communication) on universal robot pendant.  

### Host machine
1. Connect an Ethernet cable between the host computer and the Ethernet port of UR3e's controller
2. Set the network configuration as below  
    <img src=image/network.png width=280>  
    - The ros node expects to reach the robot at the IP `10.0.2.2`. You can change the IP with pendant  
    - This IP is set to the `robot_ip` argument as below  
        ```bash
        ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=10.0.2.2 launch_rviz:=false
        ```
3. Build the docker environment as below  
    ```bash
    git clone https://github.com/UOsaka-Harada-Laboratory/ur3e_ros2_tutorials.git --recursive --depth 1 && cd ur3e_ros2_tutorials && COMPOSE_DOCKER_CLI_BUILD=1 DOCKER_BUILDKIT=1 docker compose build --no-cache --parallel  
    ```

## Usage with docker

### Using utility scripts

1. Build and run the docker environment
    ```bash
    docker compose up
    ```
2. Enter the docker container
    ```bash
    xhost + && docker exec -it ur3e_humble_container bash
    ```

### Simulation

3. Run a demonstration on the host machine  

    - Visualizing the model
        ```bash
        ros2 launch ur_description view_ur.launch.py ur_type:=ur3e
        ```
        <img src=image/rviz.gif width=680>  

    - Executing the moveit GUI
        ```bash
        ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=127.0.0.1 use_fake_hardware:=true launch_rviz:=false
        ```  
        ```bash
        ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e launch_rviz:=true
        ```  
        <img src=image/moveit_gui_sim.gif width=680>  

    - Executing a motion with a Python script and scaled_joint_trajectory_controller
        ```bash
        ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=127.0.0.1 use_fake_hardware:=true initial_joint_controller:=scaled_joint_trajectory_controller launch_rviz:=true
        ```  
        ```bash
        ros2 run ur3e_tutorials_py hello_joint_trajectory_controller
        ```  
        or  
        ```bash
        ros2 launch ur3e_tutorials_py hello_joint_trajectory_controller.launch.py
        ```  
        <img src=image/ctrl_py_sim.gif width=680>  

    - Executing a motion with a C++ program and MoveIt
        ```bash
        ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=127.0.0.1 use_fake_hardware:=true initial_joint_controller:=scaled_joint_trajectory_controller launch_rviz:=false
        ```  
        ```bash
        ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e launch_rviz:=true
        ```  
        ```bash
        ros2 run ur3e_tutorials_cpp hello_moveit
        ```  
        or  
        ```bash
        ros2 launch ur3e_tutorials_cpp hello_moveit.launch.py
        ```  
        <img src=image/moveit_cpp_sim.gif width=680>  

### Real robot

3. Connect to the robot  
    ```bash
    ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=10.0.2.2 launch_rviz:=false
    ```
4. Run the external control script on the pendant  

5. Run a demonstration on the host machine  

    - Executing the moveit GUI
        ```bash
        ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e launch_rviz:=true
        ```  

## Author / Contributor

[Takuya Kiyokawa](https://takuya-ki.github.io/)

We always welcome collaborators!

## License

This software is released under the MIT License, see [LICENSE](./LICENSE).
