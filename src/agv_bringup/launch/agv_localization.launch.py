"""Localization subsystem launch file."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('agv_bringup')
    params_file = os.path.join(bringup_dir, 'config', 'agv_params.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='agv_localization', executable='gps_converter_node',
             name='gps_converter', parameters=[params_file, {'use_sim_time': use_sim_time}],
             output='screen'),
        Node(package='agv_localization', executable='wheel_odometry_node',
             name='wheel_odometry', parameters=[params_file, {'use_sim_time': use_sim_time}],
             output='screen'),
        Node(package='agv_localization', executable='lidar_odometry_node',
             name='lidar_odometry', parameters=[params_file, {'use_sim_time': use_sim_time}],
             output='screen'),
        Node(package='agv_localization', executable='ukf_node',
             name='ukf_localization', parameters=[params_file, {'use_sim_time': use_sim_time}],
             output='screen'),
    ])
