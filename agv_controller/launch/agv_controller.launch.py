import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('model', default_value='agv1',
                          choices=['agv1'],
                          description='parts model'),
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('parts_name', default_value=LaunchConfiguration('model'),
                          description='parts name'),
    DeclareLaunchArgument('namespace', default_value=LaunchConfiguration('parts_name'),
                          description='parts namespace'),
]

def generate_launch_description():
    test_description = get_package_share_directory('agv_sim')
    xacro_file = PathJoinSubstitution([test_description, 'urdf', 'agv_description.urdf.xacro'])
    namespace = LaunchConfiguration('namespace')
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': Command([
                'xacro', ' ', xacro_file, ' ',
                'model:=', LaunchConfiguration('model'), ' ',
                'namespace:=', namespace])},
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    agv_controller = Node(
        package='agv_controller',
        executable='agv_drive',
        name='agv_controller',
        output='screen'
    )

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    # Add nodes to LaunchDescription
    ld.add_action(robot_state_publisher)
    ld.add_action(agv_controller)

    return ld
