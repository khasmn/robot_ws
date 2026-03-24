#!/bin/bash

# Source the workspace
source ~/robot_ws/install/setup.bash

echo " Starting Unified Robot Control System..."

# Start ROSBridge
echo "Starting ROSbridge"
ros2 launch rosbridge_server rosbridge_websocket_launch.xml > /dev/null 2>&1 &
PID_ROSBRIDGE=$!

# Start the Mobile Base
echo "Starting Mobile Base Bringup"
ros2 launch ired_bringup bringup.launch.py &
PID_MOBILE=$!

# Start your Serial Bridge
echo "Starting Gantry Serial Bridge"
ros2 run gantry_commander serial_bridge &
PID_BRIDGE=$!

echo "Starting Web Server"
cd ~/robot_ws/src/gantry_commander/www
python3 -m http.server 8080 > /dev/null 2>&1 &
PID_WEB=$!

echo ""
echo " All systems running!"
echo " Open your browser and go to: http://localhost:8080"
echo " Press [Ctrl+C] to shut everything down."

trap "echo -e '\nShutting down system...'; kill $PID_ROSBRIDGE $PID_MOBILE $PID_BRIDGE $PID_WEB; exit" INT
wait