import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('gantry_commander')
    urdf_file = os.path.join(pkg_dir, 'gantry.urdf.xml')

    return LaunchDescription([
        # 1. Robot State Publisher (Processes the URDF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': Command(['xacro ', urdf_file])}]
        ),

        # 2. Joint State Publisher GUI (Provides the sliders)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui'
        ),

        # 3. RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])