#!/bin/bash

byobu new-session -d -s ur3e_moveit_real
byobu select-pane -t 0
byobu split-window -v
byobu select-pane -t 0

byobu send-keys -t 0 'xhost + && docker exec -it ur3e_container bash -it -c "roslaunch ur3e_moveit_config ur3e_moveit_planning_execution.launch sim:=false fake_execution:=false"' 'C-m'
sleep 5.
byobu send-keys -t 1 'xhost + && docker exec -it ur3e_container bash -it -c "roslaunch ur3e_moveit_config moveit_rviz.launch rviz_config:=/catkin_ws/src/universal_robot/ur3e_moveit_config/launch/moveit.rviz"' 'C-m'

byobu attach -t ur3e_moveit_real