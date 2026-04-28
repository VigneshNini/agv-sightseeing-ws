"""Simulation launch file."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    sim_dir = get_package_share_directory('agv_simulation')
    bringup_dir = get_package_share_directory('agv_bringup')
    urdf_file = os.path.join(sim_dir, 'urdf', 'agv_robot.urdf.xacro')
    world_file = os.path.join(sim_dir, 'worlds', 'sightseeing_park.world')

    robot_description = Command(['xacro ', urdf_file])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description, 'use_sim_time': True}]),
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'],
            output='screen'),
        Node(package='gazebo_ros', executable='spawn_entity.py',
             arguments=['-topic', 'robot_description', '-entity', 'agv_robot'],
             output='screen'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', 'agv_full.launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items()),
    ])
