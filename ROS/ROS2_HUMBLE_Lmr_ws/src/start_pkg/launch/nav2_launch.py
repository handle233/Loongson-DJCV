import os  # 引入 Python 的 os 模块，用于处理文件路径操作

from launch import LaunchDescription  # LaunchDescription 是 ROS2 启动系统的核心类，用于描述要启动哪些动作

from launch.actions import IncludeLaunchDescription, TimerAction  # IncludeLaunchDescription 用于包含其他 launch 文件；TimerAction 用于延迟执行某些启动

from launch.launch_description_sources import PythonLaunchDescriptionSource  # 指定被包含的 launch 文件是 Python 类型（.py）

from ament_index_python.packages import get_package_share_directory  # 用于获取某个 ROS2 包的 share 目录路径（即安装路径）


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

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'rviz_launch.py')
        )
    )

    local_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'local_launch.py')
        )
    )

    # control_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'my_control_launch.py')
    #     )
    # )

    delayed_rviz_launch = TimerAction(
        period=1.0,  
        actions=[rviz_launch]  
    )

    delayed_local_launch = TimerAction(
        period=2.0, 
        actions=[local_launch]  
    )

    # delayed_control_launch = TimerAction(
    #     period=3.0,  
    #     actions=[control_launch]  
    # )

    return LaunchDescription([
        imu_launch,
        lidar_launch,
        cmd_launch,
        car_launch,
        delayed_rviz_launch,
        delayed_local_launch,
        # delayed_control_launch
        
    ])
