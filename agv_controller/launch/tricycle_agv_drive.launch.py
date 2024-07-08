from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    test_description = get_package_share_directory('agv_sim')
    xacro_file = PathJoinSubstitution([test_description, 'urdf', 'agv_description.urdf.xacro'])
    return LaunchDescription([
        Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': False,
            'robot_description': Command([
                'xacro', ' ', xacro_file, ' ',
                'model:=', 'agv1', ' ',
                'namespace:=', 'agv1'])},
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    ),
        Node(
            package='agv_controller',
            executable='tricycle_agv_drive',
            name='tricycle_agv_drive',
            output='screen',
            parameters=[{
                'max_wheel_accel': 0.1,
                'max_wheel_decel': 0.1,
                'max_wheel_speed_tol': 1.0,
                # 'wheel_separation': 0.9,
                'drive_wheel_radius': 0.31,
                # 'front_wheel_radius': 0.25,
                'publish_odom': True,
                'publish_wheel_tf': True,
                'publish_wheel_joint_state': True
            }]
        ),
    ])
