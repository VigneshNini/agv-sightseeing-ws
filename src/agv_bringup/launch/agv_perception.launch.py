"""Perception subsystem launch file."""
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
        Node(package='agv_perception', executable='lidar_obstacle_detector',
             name='lidar_obstacle_detector', parameters=[params_file, {'use_sim_time': use_sim_time}],
             output='screen'),
        Node(package='agv_perception', executable='object_tracker_node',
             name='object_tracker', parameters=[params_file, {'use_sim_time': use_sim_time}],
             output='screen'),
        Node(package='agv_perception', executable='camera_perception_node',
             name='camera_perception', parameters=[params_file, {'use_sim_time': use_sim_time}],
             output='screen'),
    ])
