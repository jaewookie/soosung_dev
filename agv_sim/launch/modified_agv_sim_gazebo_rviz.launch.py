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
# Authors: Joep Tool

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch.conditions import IfCondition

from launch_ros.actions import Node

def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('agv_sim'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    rviz_display_config_file = os.path.join(
        get_package_share_directory('agv_sim'),
        'rviz',
        'agv.rviz')

    world = os.path.join(
        get_package_share_directory('agv_sim'),
        'worlds',
        'big_world.world'
        # 'empty_world.world'
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_display_config_file],
        output='screen')

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'modified_robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_agv_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'modified_spawn_agv.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    ld = LaunchDescription()

    # Add the commands to the launch description
    # ld.add_action(start_gazebo_server_cmd)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_agv_cmd)
    # ld.add_action(
    #     Node(
    #         package='agv_controller',
    #         executable='tricycle_agv_drive',
    #         name='tricycle_agv_drive',
    #         output='screen',
    #         parameters=[{
    #             'max_wheel_accel': 0.1,
    #             'max_wheel_decel': 0.1,
    #             'max_wheel_speed_tol': 1.0,
    #             # 'wheel_separation': 0.9,
    #             'drive_wheel_radius': 0.31,
    #             # 'front_wheel_radius': 0.25,
    #             'publish_odom': True,
    #             'publish_wheel_tf': True,
    #             'publish_wheel_joint_state': True
    #         }]
    #     ),
    # )
    ld.add_action(rviz2)

    return ld
