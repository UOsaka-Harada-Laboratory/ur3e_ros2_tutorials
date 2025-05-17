# ur3e_tutorials

[![support level: community](https://img.shields.io/badge/support%20level-community-lightgray.svg)](http://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)
[![license: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
![repo size](https://img.shields.io/github/repo-size/UOsaka-Harada-Laboratory/ur3e_tutorials)

- ROS2 package for Universal Robots ur3e tutorial.
  - [ur3e_tutorials](/catkin_ws/src/ur3e_tutorials): A tutorial package to execute simple demonstrations.
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
    - The ros node expects to reach the robot at the IP `172.32.1.148`. You can change the IP with pendant  
    - This IP is set to the `robot_ip` argument as below  
        ```bash
        roslaunch ur_robot_driver ur3e_bringup.launch robot_ip:=172.32.1.148
        ```
3. Build the docker environment as below  
    ```bash
    sudo apt install byobu && git clone git@github.com:UOsaka-Harada-Laboratory/ur3e_tutorials.git --depth 1 && cd ur3e_tutorials && COMPOSE_DOCKER_CLI_BUILD=1 DOCKER_BUILDKIT=1 docker compose build --no-cache --parallel  
    ```

## Usage with docker

### Using utility scripts

1. Build and run the docker environment
   - Create and start docker containers in the initially opened terminal
        ```bash
        docker compose up
        ```

### Simulation

2. Run a demonstration on the host machine  

   - Visualizing the model
        ```bash
        ./utils/ur3e_rviz.sh
        ```
        <!-- <img src=image/ur3e_rviz.sh.gif height=200> -->

   - Executing the moveit
        ```bash
        ./utils/ur3e_moveit_sim.sh
        ```
        <!-- <img src=image/ur3e_moveit_sim.sh.gif height=200> -->

   - Executing a wiggle demonstration
        ```bash
        ./utils/ur3e_wiggle_fake.sh
        ```
        <!-- <img src=image/ur3e_wiggle.sh.gif height=200> -->

   - Executing a pick-and-place demonstration
        ```bash
        ./utils/ur3e_pp_fake.sh
        ```
        <!-- <img src=image/ur3e_pp_fake.sh.gif height=200> -->

### Real robot

2. Connect to the robot  
    ```bash
    xhost + && docker exec -it ur3e_container bash -it -c "roslaunch ur_robot_driver ur3e_bringup.launch robot_ip:=172.32.1.148"
    ```
3. Run the external control script on the pendant  

4. Run a demonstration on the host machine  

   - Executing the moveit
        ```bash
        ./utils/ur3e_moveit_real.sh
        ```

   - Executing a wiggle demonstration
        ```bash
        ./utils/ur3e_wiggle.sh
        ```

   - Executing a pick-and-place demonstration
        ```bash
        ./utils/ur3e_pp.sh
        ```

### Manually execute commands

1. Build and run the docker environment
   - Create and start docker containers in the initially opened terminal
        ```bash
        docker compose up
        ```
   - Execute the container in another terminal
        ```bash
        xhost + && docker exec -it ur3e_container bash
        ```

2. Run a demonstration in the container  
    ```bash
    byobu
    ```
    - First command & F2 to create a new window & Second command ...
    - Ctrl + F6 to close the selected window

## Author / Contributor

[Takuya Kiyokawa](https://takuya-ki.github.io/)

We always welcome collaborators!

## License

This software is released under the MIT License, see [LICENSE](./LICENSE).
