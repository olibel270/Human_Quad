#!/bin/sh
tmux new-session -d 'roslaunch mavros px4.launch'
tmux split-window -v 'python3.8 ~/Human_Quad/positioning/positioning_pub.py'
tmux split-window -h 'rostopic echo /mavros/local_position/pose'
tmux new-window 'mutt'
tmux -2 attach-session -d
