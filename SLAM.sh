#!/bin/bash

# Load ROS2
source ~/turtlebot3_ws/install/setup.bash

SESSION="slam"

# Kill session cũ nếu có
tmux kill-session -t $SESSION 2>/dev/null

# Tạo session mới
tmux new-session -d -s $SESSION

# Cửa sổ 1 – main.py
tmux rename-window -t $SESSION:0 "main"
tmux send-keys -t $SESSION:0 "source ~/turtlebot3_ws/install/setup.bash" C-m
tmux send-keys -t $SESSION:0 "python3 /home/amr/turtlebot3_ws/src/main/main.py" C-m

# Cửa sổ 2 – cartographer
tmux new-window -t $SESSION -n "slam"
tmux send-keys -t $SESSION:1 "source ~/turtlebot3_ws/install/setup.bash" C-m
tmux send-keys -t $SESSION:1 "ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=false" C-m

# Cửa sổ 3 – bringup
tmux new-window -t $SESSION -n "bringup"
tmux send-keys -t $SESSION:2 "source ~/turtlebot3_ws/install/setup.bash" C-m
tmux send-keys -t $SESSION:2 "ros2 launch turtlebot3_bringup robot.launch.py" C-m

# Mở tmux GUI
tmux attach-session -t $SESSION
