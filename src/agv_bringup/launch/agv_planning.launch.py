"""Planning subsystem launch file."""
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
        Node(package='agv_planning', executable='global_route_manager',
             name='global_route_manager', parameters=[params_file, {'use_sim_time': use_sim_time}],
             output='screen'),
        Node(package='agv_planning', executable='hybrid_astar_planner',
             name='hybrid_astar_planner', parameters=[params_file, {'use_sim_time': use_sim_time}],
             output='screen'),
        Node(package='agv_planning', executable='mpc_local_planner',
             name='mpc_local_planner', parameters=[params_file, {'use_sim_time': use_sim_time}],
             output='screen'),
        Node(package='agv_planning', executable='velocity_profiler',
             name='velocity_profiler', parameters=[params_file, {'use_sim_time': use_sim_time}],
             output='screen'),
        Node(package='agv_planning', executable='behavior_tree_node',
             name='behavior_tree', parameters=[params_file, {'use_sim_time': use_sim_time}],
             output='screen'),
    ])
