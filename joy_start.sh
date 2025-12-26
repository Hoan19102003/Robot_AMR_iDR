#!/bin/bash

# ===========================
# Load ROS2 workspace
# ===========================
source ~/turtlebot3_ws/install/setup.bash

# ===========================
# Chạy ROS2 Joy Node
# ===========================
(
    echo -e "${CYAN}[ROS_JOY] Running ros2_joy.py...${NC}"
    /usr/bin/python3 /home/amr/turtlebot3_ws/joy/ros2_joy.py
) &

# ===========================
# Chạy Gamepad Teleop
# ===========================
(
    echo -e "${GREEN}[GAMEPAD] Running gamepad_teleop.py...${NC}"
    /usr/bin/python3 /home/amr/turtlebot3_ws/joy/gamepad_teleop.py
) &

# ===========================
# Chờ tất cả tiến trình
# ===========================
wait
