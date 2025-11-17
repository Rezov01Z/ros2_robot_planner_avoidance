#!/usr/bin/env python3

import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch arguments
    goal_x_arg = DeclareLaunchArgument(
        'goal_x', default_value='2.0',
        description='Goal position X coordinate'
    )
    goal_y_arg = DeclareLaunchArgument(
        'goal_y', default_value='-1.0',
        description='Goal position Y coordinate'
    )

    # Gazebo simulation
    gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch'),
            '/turtlebot3_world.launch.py'
        ])
    )

    # Smart Navigation node
    smart_navigation_node = Node(
        package='obstacle_avoidance_tb3',
        executable='smart_navigation',
        name='smart_navigation',
        output='screen',
        parameters=[{
            'goal_x': LaunchConfiguration('goal_x'),
            'goal_y': LaunchConfiguration('goal_y')
        }]
    )

    # RViz configuration
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('obstacle_avoidance_tb3'),
        'config',
        'smart_navigation.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        goal_x_arg,
        goal_y_arg,
        gazebo_world,
        smart_navigation_node,
        rviz_node
    ])