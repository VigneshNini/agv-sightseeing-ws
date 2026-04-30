#!/usr/bin/env python3
"""
Automated LiDAR Test Launch File

Launches:
  1. fake_lidar_publisher.py  — publishes /scan at 10 Hz
  2. static_transform_publisher — broadcasts laser_frame TF
  3. RViz2                    — pre-configured to show LaserScan

Usage (after building workspace):
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    ros2 launch agv_bringup fake_lidar_test.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    # Path to the RViz2 config file
    rviz_config = os.path.join(
        get_package_share_directory('agv_bringup'),
        'config',
        'lidar_rviz.rviz'
    )

    # Path to the fake lidar publisher script
    ws_dir = os.path.join(
        get_package_share_directory('agv_bringup'),
        '..', '..', '..', '..', '..'
    )
    fake_lidar_script = os.path.join(
        os.path.abspath(ws_dir), 'scripts', 'fake_lidar_publisher.py'
    )

    return LaunchDescription([

        LogInfo(msg='============================================'),
        LogInfo(msg='  AGV Fake LiDAR Test — Starting Up'),
        LogInfo(msg='  Topic: /scan  |  Frame: laser_frame'),
        LogInfo(msg='============================================'),

        # 1. Static TF: world -> laser_frame
        #    Needed so RViz2 knows where the LiDAR is in space
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_tf_publisher',
            arguments=['0', '0', '0.3', '0', '0', '0', 'world', 'laser_frame'],
            output='screen'
        ),

        # 2. Fake LiDAR publisher (Python script as process)
        ExecuteProcess(
            cmd=['python3', fake_lidar_script],
            name='fake_lidar_publisher',
            output='screen'
        ),

        # 3. RViz2 — delayed by 2 seconds to let the publisher start first
        TimerAction(
            period=2.0,
            actions=[
                LogInfo(msg='Starting RViz2...'),
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config],
                    output='screen'
                ),
            ]
        ),
    ])
