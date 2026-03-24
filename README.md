***1.Install Package***
```
sudo apt update
sudo apt install ros-jazzy-moveit \
                 ros-jazzy-moveit-ros-visualization \
                 ros-jazzy-joint-state-publisher-gui \
                 ros-jazzy-xacro \
                 ros-jazzy-rosbridge-server \
                 python3-serial
```


build the workspace
```
cd ~/robot_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

***2.Hardware Configuration***
```
lsusb
```
Look for the line corresponding to the Arduino Due.
Example: ID 2341:003d (2341 is Vendor & 003d is Product).

Create the Udev Rule using Nano:
```
sudo nano /etc/udev/rules.d/99-gantry.rules
```
Create this rule (Note: the symlink must be arduino_gantry to match the serial bridge code):
```
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="003d", SYMLINK+="arduino_gantry", MODE="0666"
```
Change idVendor and idProduct to match your specific IDs.
SYMLINK+="arduino_gantry": This creates a permanent alias at /dev/arduino_gantry.
MODE="0666": This grants read/write permissions to all users.

Reload and Trigger:
```
sudo udevadm control --reload-rules
sudo udevadm trigger
```

***3.Launching the System***

You can run the system using the automated script or by manually launching each component in separate terminals.

Option A: Automated Script
This script starts the ROS Bridge, the iRED Mobile Base hardware, the Gantry Serial Bridge, and the Web Server automatically.

Open a terminal and run the background services:
```
cd ~/robot_ws
./start_system.sh
```
Open a second terminal for MoveIt:
```
ros2 launch gantry_moveit_config demo.launch.py
```

Option B: Manual Terminal Execution
If you are not using the shell script, you must open four separate terminals. Source your workspace (source ~/robot_ws/install/setup.bash) in each terminal before running the commands.

Terminal 1 (ROS Bridge Websocket):
```
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

Terminal 2 (Local Web Server):
```
cd <path_to_directory_containing_index.html>
python3 -m http.server 8080
```

Terminal 3 (Serial Bridge):
```
ros2 run gantry_commander serial_bridge
```

Terminal 4 (MoveIt):
```
ros2 launch gantry_moveit_config demo.launch.py
```
***4.Web Dashboard Control***

Access the UI at http://localhost:8080 (or http://<YOUR_IP>:8080 on otherdevice).

1. Gantry Control

Initialize: Click HOME first to calibrate the stepper motors.

Jogging: Use the Arrow D-Pad for precise X/Y movements.

MoveIt: Click STREAM to allow MoveIt trajectories to pass to the hardware.


2. Omni Mobile Base (Cruise Control)

Incremental Speed: Click the direction arrows (↑, ↓, ←, →) to increase speed by 0.05m/s per click.

Rotation: Use the ↺ / ↻ buttons to increase angular velocity by 0.1 rad/s.

Brake: Click the Red STOP button to instantly reset all velocities to 0.00.

3. PID Tuning Deck

Select the target motor (FL, FR, BL, BR) from the dropdown.

Enter your Kp, Ki, Kd values.

Click Apply Parameters. Check the System Telemetry log for a "SUCCESS" confirmation.

***Developer: Kha

Instagram: @kha_smn

Emergency Call: 061-760-3050 (ONLY FOR HARDWARE EMERGENCIES)***
