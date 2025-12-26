#!/bin/bash

# Load ROS2
source ~/turtlebot3_ws/install/setup.bash

SESSION="navigation"

# Xóa session cũ nếu có
tmux kill-session -t $SESSION 2>/dev/null

# Tạo session mới với window 0 = main
tmux new-session -d -s $SESSION -n main
tmux send-keys -t $SESSION:0 "source ~/turtlebot3_ws/install/setup.bash" C-m
tmux send-keys -t $SESSION:0 "python3 /home/amr/turtlebot3_ws/src/main/main.py" C-m

# Window 1 = navigation2
tmux new-window -t $SESSION:1 -n navigation
tmux send-keys -t $SESSION:1 "source ~/turtlebot3_ws/install/setup.bash" C-m
tmux send-keys -t $SESSION:1 "ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=\$HOME/map.yaml" C-m

# Window 2 = bringup
tmux new-window -t $SESSION:2 -n bringup
tmux send-keys -t $SESSION:2 "source ~/turtlebot3_ws/install/setup.bash" C-m
tmux send-keys -t $SESSION:2 "ros2 launch turtlebot3_bringup robot.launch.py" C-m

# Mở tmux GUI
tmux attach-session -t $SESSION
