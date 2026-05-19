from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import yaml


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_to_laserscan',
            executable='gimbal_node',
            name='gimbal',
            parameters=[{'child_frame': 'base_gimbal', 
                         'parent_frame': 'base_link',
                         'imu_topic': 'imu_in', 
                         'transform_tolerance': 0.01}]
        ),
    ])
