# Copyright 2021 Clearpath Robotics, Inc.
# Copyright 2023 Bernd Pfrommer
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)
# @author Bernd Pfrommer (bernd@pfrommer.us)

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution
from launch.substitutions.launch_configuration import LaunchConfiguration

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
    xacro_file = PathJoinSubstitution([test_description, 'urdf', 'modified_agv_description.urdf.xacro'])
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
    rviz_display_config_file = os.path.join(
        get_package_share_directory('agv_sim'),
        'rviz',
        'agv.rviz')

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_display_config_file],
        output='screen')

    # world_tf_publisher = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments=['--x', '0', '--y', '0', '--z', '1', '--yaw', '0',
    #                '--pitch', '0', '--roll', '0',
    #                '--frame-id', 'world', '--child-frame-id', 'base'])

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
    ld.add_action(joint_state_publisher)
    ld.add_action(rviz2)
    # ld.add_action(agv_controller)
    # ld.add_action(world_tf_publisher)

    return ld
