import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    param_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(
            get_package_share_directory('ired_bringup'),
            'param',
            'ired.yaml'))
    laser_param_dir = LaunchConfiguration(
        'laser_param_dir',
        default=os.path.join(
           get_package_share_directory('ired_bringup'),
            'param',
            'laser_config.yaml'))
    laser_scan_matcher_dir = LaunchConfiguration(
        'laser_scan_matcher_dir',
        default=os.path.join(
           get_package_share_directory('ired_bringup'),
            'param',
            'laser_scan_matcher.yaml'))
    urdf_file = os.path.join(
        get_package_share_directory('ired_bringup'),
        'urdf',
        'ired.urdf.xacro')
    rplidar_launch = LaunchConfiguration(
        'rplidar_launch',
        default=os.path.join(
            get_package_share_directory('rplidar_ros'), 'launch'))
    return LaunchDescription([
        LogInfo(msg=['Starting iRED Bringup...']),
        DeclareLaunchArgument(
            'param_dir',
            default_value=param_dir,
            description='Specifying parameter direction'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', str(urdf_file)]),
                'publish_frequency': 10.0,
            }]),
        Node(
            package='ired_bringup',
            executable='forward_kinematics_node',
            parameters=[param_dir],
            output='screen'),
        Node(
            package='ired_bringup',
            executable='inverse_kinematics_node',
            parameters=[param_dir],
            output='screen'),
        Node(
            package='ired_bringup',
            executable='imu_node',
            parameters=[param_dir],
            output='screen'),
        Node(
            package='ired_bringup',
            executable='odom_node',
            parameters=[param_dir],
            output='screen'),
        Node(
            package='ired_bringup',
            executable='iredcr_modbus_node',
            parameters=[param_dir],
            output='screen'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([rplidar_launch, '/rplidar_a1_launch.py']),
            launch_arguments={
                'serial_port': '/dev/ttyLiDAR',
                'serial_baudrate': '115200',
                'frame_id': 'base_scan'}.items()),
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            parameters=[laser_param_dir],
            output='screen',
            arguments=['--ros-args', '--log-level', 'error']),
        Node(
            package='ros2_laser_scan_matcher',
            executable='laser_scan_matcher',
            parameters=[laser_scan_matcher_dir],
            output='screen')
    ])
