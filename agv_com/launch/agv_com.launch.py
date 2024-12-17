import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    
    agv_rs_imu = Node(
        package='agv_com',
        executable='agv_rs_imu',
        name='agv_rs_imu',
        output='screen',
    )

    agv_can_bus = Node(
        package='agv_com',
        executable='agv_can_bus',
        name='agv_can_bus',
        output='screen',
    )
    
    ros2socketcan = Node(
        package='ros2socketcan_bridge',
        executable='ros2socketcan',
        name='ros2socketcan',
        output='screen',
    )

    # Define LaunchDescription variable
    ld = LaunchDescription()
    # Add nodes to LaunchDescription
    ld.add_action(ros2socketcan)
    # ld.add_action(agv_rs_imu)
    ld.add_action(agv_can_bus)

    return ld
