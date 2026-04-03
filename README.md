# Gantry Robot Workspace Setup

## 1. Install Packages

```bash
sudo apt update
sudo apt install ros-jazzy-moveit \
                 ros-jazzy-moveit-ros-visualization \
                 ros-jazzy-joint-state-publisher-gui \
                 ros-jazzy-xacro \
                 ros-jazzy-rosbridge-server \
                 python3-serial \
                 ros-jazzy-librealsense2*\
                 ros-jazzy-librealsense2-camera\
                 ros-jazzy-librealsense2-tools
```

Build the workspace:

```bash
cd ~/robot_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Optionally to only build the gantry packages:

```bash
cd ~/robot_ws
colcon build --packages-select gantry_bringup gantry_commander gantry_moveit_config yolo_realsense --symlink-install
```

## 2. Hardware Configuration (Udev Rules)

Create the Udev rules file:

```bash
sudo nano /etc/udev/rules.d/99-gantry.rules
```

Add:

```bash
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="003d", SYMLINK+="arduino_gantry", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", SYMLINK+="gripper_gantry", MODE="0666"
```

- `SYMLINK+="arduino_gantry"` creates a permanent alias at `/dev/arduino_gantry`.
- `MODE="0666"` grants read/write permissions to all users.

Reload rules:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## 3. Create Virtual Environment

Create and install Python dependencies:

```bash
cd ~/robot_ws
python3 -m venv venv
source venv/bin/activate
pip install ultralytics pyserial
```

Edit the activation script:

```bash
nano venv/bin/activate
```

Add this line at the end of the file:

```bash
export PYTHONPATH="/home/<user>/robot_ws/venv/lib/python3.12/site-packages:$PYTHONPATH"
```

Replace `<user>` with your Linux username.

Activate the virtual environment before launching the system:

```bash
source ~/robot_ws/venv/bin/activate
```

## 4. Launching the System

You can start the system in two ways.

### Option A: Gantry Bringup (Recommended)

Start core services:

```bash
source ~/robot_ws/install/setup.bash
ros2 launch gantry_bringup bringup.launch.py
```

### Option B: Manual Launch

If you are not using `gantry_bringup`, open separate terminals and run each component manually.
In each terminal, run:

```bash
source ~/robot_ws/install/setup.bash
```

Terminal 1 (Camera Node):

```bash
ros2 launch realsense2_camera rs_align_depth.py
```

Terminal 2 (MoveIt):

```bash
ros2 launch gantry_moveit_config demo.launch.py
```

Terminal 3 (Serial Bridge):

```bash
ros2 run gantry_commander serial_bridge_cli
```

Terminal 4 (YOLO Node):

```bash
ros2 run yolo_realsense detector
```

Terminal 5 (YOLO to MoveIt):

```bash
ros2 run yolo_realsense yolo_to_moveit
```
