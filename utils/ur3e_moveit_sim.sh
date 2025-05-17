#!/bin/bash

byobu new-session -d -s ur3e_moveit_sim
byobu select-pane -t 0
byobu split-window -v
byobu select-pane -t 0

byobu send-keys -t 0 'xhost + && docker exec -it ur3e_container bash -it -c "roslaunch ur3e_moveit_config demo.launch"' 'C-m'

byobu attach -t ur3e_moveit_sim