from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
import launch_ros
import launch
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value = 'False')

    cartographer_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_node',
        parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments = [
            '-configuration_directory', os.path.join(get_package_share_directory("lidar_cartographer"),"config"),
            '-configuration_basename', 'mycar.lua'],
        # remappings = [
        #     ('scan', '/scan'),
        #     ('odom', '/odom')],
        output = 'screen'
    )

    cartographer_occupancy_grid_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_occupancy_grid_node',
        parameters = [
            {'use_sim_time': False},
            {'resolution': 0.05}],
    )

    return LaunchDescription([
        use_sim_time_arg,
        cartographer_node,
        cartographer_occupancy_grid_node,
    ])

 


