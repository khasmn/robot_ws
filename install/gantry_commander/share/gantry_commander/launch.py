import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_dir = get_package_share_directory('gantry_commander')
    urdf_path = os.path.join(package_dir, 'gantry.urdf.xml')
    config_path = os.path.join(package_dir, 'controllers.yaml')

    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher', 
             parameters=[{'robot_description': robot_desc}]),
        Node(package='controller_manager', executable='ros2_control_node',
             parameters=[{'robot_description': robot_desc}, config_path]),
        Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster']),
        Node(package='controller_manager', executable='spawner', arguments=['arm_controller']),
        Node(package='tf2_ros', executable='static_transform_publisher', arguments=['0','0','0','0','0','0','world','base_link'])
    ])