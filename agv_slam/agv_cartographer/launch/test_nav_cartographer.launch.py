#!/usr/bin/env python3
#
# Copyright 2022 ROBOTIS CO., LTD.
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
# Author: Darby Lim

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start_rviz = LaunchConfiguration('start_rviz')
    use_sim = LaunchConfiguration('use_sim')
    # laser_params_file = LaunchConfiguration('laser_params_file')

    # laser_params_file = LaunchConfiguration(
    #     'laser_params_file',
    #     default=PathJoinSubstitution(
    #         [
    #             FindPackageShare('agv_cartographer'),
    #             'config',
    #             'pointcloud_to_laserscan_params.yaml'
    #         ]
    #     )
    # )

    cartographer_config_dir = PathJoinSubstitution(
        [
            FindPackageShare('agv_cartographer'),
            'config',
        ]
    )
    configuration_basename = LaunchConfiguration('configuration_basename')

    resolution = LaunchConfiguration('resolution')


    return LaunchDescription([

        # DeclareLaunchArgument(
        #     'laser_params_file',
        #     default_value=laser_params_file,
        #     description='Full path to the ROS2 parameters file to use for all launched nodes'),

        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Start robot in Gazebo simulation'),

        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Full direction of config file'),

        DeclareLaunchArgument(
            'configuration_basename',
            default_value='test_nav_3d.lua',
            description='Name of lua file for cartographer'),

        DeclareLaunchArgument(
            'resolution',
            default_value='0.05',
            description='Resolution of a grid cell of occupancy grid'),

        # Node(
        #     package='pointcloud_to_laserscan',
        #     executable='pointcloud_to_laserscan_node',
        #     name='pointcloud_to_laserscan',
        #     output='screen',
        #     parameters=[laser_params_file],
        #     remappings=[
        #         ('/cloud_in', '/points2'),
        #         ('/scan', '/scan_s')
        #     ]
        # ),

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename]),

        # Node(
        #     package='cartographer_ros',
        #     executable='cartographer_occupancy_grid_node',
        #     output='screen',
        #     parameters=[{'use_sim_time': use_sim}],
        #     arguments=['-resolution', resolution])
    ])
