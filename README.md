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

***2.Defaulting the port for Arduino so no hard code***
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

***3.Run the system***

You can run the system using the automated script or by manually launching each component in separate terminals.

Option A: Automated Script
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

Once the system is running, the terminal UI is no longer used. All controls are handled via the web browser.

    1.Open a web browser on the host machine and go to: http://localhost:8080

    2.To control the gantry from a smartphone or another computer on the same Wi-Fi network, find the host machine's IP address by running hostname -I in the terminal. Navigate to http://<YOUR_IP_ADDRESS>:8080 on the remote device.

    3.Click "HOME GANTRY" first.

    4.To use MoveIt, click "START MOVEIT" to unlock the trajectory stream.

    5.To manually move the gantry, use the Virtual Joystick on the screen (Ensure MoveIt streaming is off before jogging).

***Any Question Contact Kha IG:kha_smn if urgen call:0617603050 #CALL WHEN EMERGENCY ONLY***
