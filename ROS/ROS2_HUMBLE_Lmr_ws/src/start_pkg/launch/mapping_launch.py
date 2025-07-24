from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('imu_pkg'), 'launch', 'imu_launch.py')
        )
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('lidar_pkg'), 'launch', 'lidar.launch.py')
        )
    )

    cmd_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('cmd_vel_tcp_send'), 'launch', 'launch.py')
        )
    )

    car_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('car_description'), 'launch', 'launch.py')
        )
    )

    cartographer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('lidar_cartographer'), 'launch', 'cartographer_launch.py')
        )
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('lidar_cartographer'), 'launch', 'rviz_launch.py')
        )
    )

    return LaunchDescription([
        imu_launch,
        lidar_launch,
        cmd_launch,
        car_launch,
        cartographer_launch,
        rviz_launch
    ])
