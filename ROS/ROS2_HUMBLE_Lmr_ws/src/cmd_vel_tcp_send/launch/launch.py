from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cmd_vel_tcp_send',  # 功能包名称
            executable='cmd_vel_tcp_node',            # 可执行文件名称
            output='screen'                       # 输出到终端
            # name='another_node_name',             # 可选
            # parameters=[                          # 可选：传递参数
            #     # {'param_name': 'param_value'}
            # ]
        ),
    ])