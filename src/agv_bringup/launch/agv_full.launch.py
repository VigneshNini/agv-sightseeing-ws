"""Full AGV system bringup launch file."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             GroupAction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('agv_bringup')
    params_file = os.path.join(bringup_dir, 'config', 'agv_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    namespace = LaunchConfiguration('namespace', default='agv')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock')

    declare_namespace = DeclareLaunchArgument(
        'namespace', default_value='agv',
        description='Robot namespace')

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'agv_localization.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items())

    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'agv_perception.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items())

    planning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'agv_planning.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items())

    safety_node = Node(
        package='agv_safety',
        executable='safety_arbiter_node',
        name='safety_arbiter',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        output='screen')

    tour_manager_node = Node(
        package='agv_tour_manager',
        executable='tour_manager_node',
        name='tour_manager',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        output='screen')

    dashboard_node = Node(
        package='agv_dashboard',
        executable='web_bridge_node',
        name='web_bridge',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        output='screen')

    return LaunchDescription([
        declare_use_sim_time,
        declare_namespace,
        localization_launch,
        TimerAction(period=2.0, actions=[perception_launch]),
        TimerAction(period=4.0, actions=[planning_launch]),
        TimerAction(period=6.0, actions=[safety_node]),
        TimerAction(period=7.0, actions=[tour_manager_node]),
        TimerAction(period=8.0, actions=[dashboard_node]),
    ])
