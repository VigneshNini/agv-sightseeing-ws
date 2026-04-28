#!/usr/bin/env python3
"""Gazebo simulation launch file for AGV Sightseeing Vehicle."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                             IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node


def generate_launch_description():
    sim_dir = get_package_share_directory('agv_simulation')
    bringup_dir = get_package_share_directory('agv_bringup')

    urdf_file = os.path.join(sim_dir, 'urdf', 'agv_robot.urdf.xacro')
    world_file = os.path.join(sim_dir, 'worlds', 'sightseeing_park.world')
    params_file = os.path.join(sim_dir, 'config', 'gazebo_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', urdf_file,
        ' use_sim:=true'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation clock'),
        DeclareLaunchArgument('x_pose', default_value='0.0',
                              description='Initial X position'),
        DeclareLaunchArgument('y_pose', default_value='0.0',
                              description='Initial Y position'),

        SetEnvironmentVariable('GAZEBO_MODEL_PATH',
                               os.path.join(sim_dir, 'models')),

        # Start Gazebo server
        ExecuteProcess(
            cmd=['gzserver', '--verbose', world_file,
                 '-s', 'libgazebo_ros_init.so',
                 '-s', 'libgazebo_ros_factory.so'],
            output='screen'),

        # Start Gazebo client (GUI)
        ExecuteProcess(
            cmd=['gzclient'],
            output='screen'),

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description_content,
                'use_sim_time': use_sim_time,
            }]),

        # Spawn robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_agv',
            output='screen',
            arguments=[
                '-topic', '/robot_description',
                '-entity', 'agv_robot',
                '-x', x_pose,
                '-y', y_pose,
                '-z', '0.3',
            ]),

        # Launch full AGV stack in simulation mode
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', 'agv_full.launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items()),
    ])
