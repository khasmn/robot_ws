***1.Install Package***
```
sudo apt update
sudo apt install ros-jazzy-moveit \
                 ros-jazzy-moveit-ros-visualization \
                 ros-jazzy-joint-state-publisher-gui \
                 ros-jazzy-xacro \
                 python3-serial
```


build the workspace
```
cd ~/robot_ws
rosdep install --from-paths src --ignore-src -r -y #Installs missing pieces
colcon build --symlink-install
source install/setup.bash
```


***2.Defualting the port for arduino so no hard code***
```
lsusb
```
Look for the line corresponding to the Arduino Due 
Example: ID 2341:003d (2341 is Vendor & 003d is Product)


Create the Udev Rule using Nano
```
sudo nano /etc/udev/rules.d/99-gantry.rules
```


then create this rule
```
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="003d", SYMLINK+="gantry", MODE="0666"
```
change idVendor as your ID and idProduct as your ID
SYMLINK+="gantry": This creates a permanent alias at /dev/gantry.
MODE="0666": This grants read/write permissions to all users.


Reload and Trigger
```
sudo udevadm control --reload-rules
sudo udevadm trigger
```

***3.Run the system***

#terminal 1
```
ros2 launch gantry_commander launch.py 
```

#termainal 2
```
ros2 launch gantry_moveit_config demo.launch.py
```

#termainl 3
```
ros2 run gantry_commander serial_bridge
```

in the serial_bridge termainal you need to type "h" to homing then type 'r' to unlock the moveit. Then in the Rviz that will open when you run
`ros2 launch gantry_moveit_config demo.launch.py` you can move the model to the position you wanted the click plan and execute the robot will move to that position


***Any Question Contact Kha IG:kha_smn if urgen call:0617603050 #CALL WHEN EMERGENCY ONLY***
