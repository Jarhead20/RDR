from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                os.path.join(
                get_package_share_directory('unity'), 'config', 'slam_toolbox_params.yaml')
            ],
            remappings=[
                ('scan', '/scan'),
                ('odom', '/odom')
            ]
        ),
        Node(
            package='unity',
            executable='camera_to_laser',
            name='camera_to_laser',
            output='screen',
        ),
        # differential odometry
        Node(
            package='unity',
            executable='differential_odom',
            name='diff_odom_node',
            output='screen',
        ),
        # Node(
        #     package='unity',
        #     executable='image_subscriber',
        #     name='image_subscriber',
        #     output='screen',
        # ),
        
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen',
        #     # arguments=['-d', os.path.join(
        #     #     get_package_share_directory('unity'), 'config', 'slam_config.rviz')]
        # )
    ])
