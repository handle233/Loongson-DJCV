#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
  # RViZ2 settings
  rviz2_config = os.path.join(
      get_package_share_directory('car_description'),
      'rviz',
      'urdf_config.rviz'
  )

  rviz2_node = Node(
      package='rviz2',
      executable='rviz2',
      name='car_description_rviz2',
      arguments=['-d',rviz2_config],
      output='screen'
  )

  #Include mini_car_description launch file
  mini_car_description_node = IncludeLaunchDescription(
      launch_description_source=PythonLaunchDescriptionSource([
          get_package_share_directory('car_description'),
          '/launch/launch.py'
      ])
  )

  # Define LaunchDescription variable
  ld = LaunchDescription()

  ld.add_action(mini_car_description_node)
  ld.add_action(rviz2_node)

  return ld