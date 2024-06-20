#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
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
# Authors: Darby Lim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    description_path = get_package_share_directory('agv_sim')
    xacro_file = PathJoinSubstitution([description_path, 'urdf', 'agv_description.urdf.xacro'])
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    namespace = LaunchConfiguration('namespace')

    print('urdf_file_name : {}'.format(xacro_file))

    return LaunchDescription([
        DeclareLaunchArgument(
            'model', default_value='agv1',
            choices=['agv1'],
            description='parts model'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
            choices=['true', 'false'],
           description='use_sim_time'),
        DeclareLaunchArgument('parts_name', default_value=LaunchConfiguration('model'),
            description='parts name'),
        DeclareLaunchArgument('namespace', default_value=LaunchConfiguration('parts_name'),
            description='parts namespace'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time,
                'robot_description': Command([
                'xacro', ' ', xacro_file, ' ',
                'model:=', LaunchConfiguration('model'), ' ',
                'namespace:=', namespace])}
                ],
        ),
    ])
