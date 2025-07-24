from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imu_pkg',
            executable='imu_pkg',
            name='imu_node',
            output='screen',
            parameters=[{
                'use_sim_time': False,
            }]
        )
    ])
