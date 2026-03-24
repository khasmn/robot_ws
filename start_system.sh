#!/bin/bash

echo " Starting Gantry Control System..."

# Start ROSBridge (The Translator)
echo "Starting ROSbridge"
ros2 launch rosbridge_server rosbridge_websocket_launch.xml > /dev/null 2>&1 &
PID_ROSBRIDGE=$!

# Start your Serial Bridge (The Hardware)
echo "Starting Serial Bridge"
ros2 run gantry_commander serial_bridge &
PID_BRIDGE=$!

# Start the Web Dashboard Server
echo "Starting Web Server"
cd ~/robot_ws/src/gantry_commander/www
python3 -m http.server 8080 > /dev/null 2>&1 &
PID_WEB=$!

echo ""
echo " All systems running!"
echo " Open your browser and go to: http://localhost:8080"
echo " Press [Ctrl+C] to shut everything down."

# Wait for Ctrl+C, then kill all 3 background processes cleanly
trap "echo -e '\nShutting down system...'; kill $PID_ROSBRIDGE $PID_BRIDGE $PID_WEB; exit" INT
wait
