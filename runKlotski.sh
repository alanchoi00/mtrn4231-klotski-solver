#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Set the robot IP address from the first command-line argument, or use default "192.168.0.100"
ROBOT_IP="${1:-192.168.0.100}"
gnome-terminal --title="UR5eDriverServer" -- bash -lc "ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=$ROBOT_IP use_fake_hardware:=false launch_rviz:=false; exec bash"

sleep 5

gnome-terminal --title="Dashboard" -- bash -lc "cd \"$SCRIPT_DIR/src/dashboard_app\" && npm install && npm run dev; exec bash"

sleep 5

source "$SCRIPT_DIR/install/setup.bash"

ros2 launch src/launch/klotski.launch.py sim:=false
