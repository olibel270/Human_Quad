#!/bin/sh
tmux new-session -d 'roslaunch mavros px4.launch'
sleep 2
tmux split-window -v 'python3.8 ~/Human_Quad/positioning/positioning_pub.py'
sleep 3
tmux split-window -h 'rostopic echo /mavros/local_position/pose'
tmux new-window 'mutt'
tmux -2 attach-session -d
