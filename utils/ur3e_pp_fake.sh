#!/bin/bash

byobu new-session -d -s ur3e_pp_fake
byobu select-pane -t 0
byobu split-window -v
byobu select-pane -t 0

byobu send-keys -t 0 'xhost + && docker exec -it ur3e_container bash -it -c "roslaunch ur3e_moveit_config demo.launch"' 'C-m'
sleep 3.
byobu send-keys -t 1 'xhost + && docker exec -it ur3e_container bash -it -c "roslaunch ur3e_tutorials pick_and_place.launch fake_execution:=true"' 'C-m'

byobu attach -t ur3e_pp_fake