import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    rviz_dir = LaunchConfiguration(
        'rviz_dir',
        default=os.path.join(
            get_package_share_directory('agv_controller'), 'launch'))

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf_file_name = 'agv.urdf'

    urdf = os.path.join(
        get_package_share_directory('agv_sim'),
        'urdf',
        urdf_file_name)

    return LaunchDescription([
        LogInfo(msg=['Execute Turtlebot3 Fake Node!!']),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([rviz_dir, '/rviz2.launch.py'])),

        Node(
            package='agv_controller',
            executable='agv_drive_fake',
            parameters=[],
            output='screen'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[urdf]),
    ])
