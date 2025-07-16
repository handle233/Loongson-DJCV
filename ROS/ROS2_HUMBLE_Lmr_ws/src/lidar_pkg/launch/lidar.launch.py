#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

'''
Parameter Description:
---
- Set laser scan directon: 
  1. Set counterclockwise, example: {'laser_scan_dir': True}
  2. Set clockwise,        example: {'laser_scan_dir': False}
- Angle crop setting, Mask data within the set angle range:
  1. Enable angle crop fuction:
    1.1. enable angle crop,  example: {'enable_angle_crop_func': True}
    1.2. disable angle crop, example: {'enable_angle_crop_func': False}
  2. Angle cropping interval setting:
  - The distance and intensity data within the set angle range will be set to 0.
  - angle >= 'angle_crop_min' and angle <= 'angle_crop_max' which is [angle_crop_min, angle_crop_max], unit is degress.
    example:
      {'angle_crop_min': 135.0}
      {'angle_crop_max': 225.0}
      which is [135.0, 225.0], angle unit is degress.
'''

def generate_launch_description():
  # LDROBOT LiDAR publisher node
  ldlidar_node = Node(
      package='lidar_pkg',
      executable='lidar_pkg',
      name='lidar_pkg',
      output='screen',
      parameters=[
        {'product_name': 'LDLiDAR_LD19'},
        {'topic_name': 'scan'},
        {'frame_id': 'laser_link'},
        # {'port_name': '/dev/ttyUSB0'},
        # {'port_baudrate': 230400},

        {'server_ip': '0'},
        {'server_port': '9999'},

        {'laser_scan_dir': True},
        {'enable_angle_crop_func': True},
        {'angle_crop_min': 170.0},
        {'angle_crop_max': 190.0}
      ]
  )


  # Define LaunchDescription variable
  ld = LaunchDescription()

  ld.add_action(ldlidar_node)


  return ld