import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'examples',
                'align_depth',
                'rs_align_depth_launch.py',
            )
        )
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gantry_moveit_config'),
                'launch',
                'demo.launch.py',
            )
        )
    )

    yolo_to_moveit = Node(
        package='yolo_realsense',
        executable='yolo_to_moveit',
        parameters=[],
        output='screen'
    )

    serial_bridge = Node(
        package='gantry_commander',
        executable='serial_bridge',
        parameters=[],
        output='screen'
    )

    yolo_node = Node(
        package='yolo_realsense',
        executable='detector',
        parameters=[],
        output='screen'
    )

    urdf_path = os.path.join(
        get_package_share_directory('gantry_bringup'),
        'gantry.urdf.xml'
    )

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', urdf_path])}],
        output='screen'
    )

    return LaunchDescription([
        camera_launch,
        robot_state_pub,
        moveit_launch,
        yolo_to_moveit,
        serial_bridge,
        yolo_node,
    ])
