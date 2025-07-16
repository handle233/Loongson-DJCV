import os

# 获取功能包的安装路径（通过ament索引机制）
from ament_index_python.packages import get_package_share_directory

# 导入launch所需的核心API
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # -------- 获取各功能包的路径 --------
    # 获取SLAM功能包路径（你写的是e05_cartographer_mapping）
    slam_pkg = get_package_share_directory("e05_cartographer_mapping")
    # 获取Nav2导航功能包路径
    nav2_pkg = get_package_share_directory("e07_nav2_bringup")
    # 获取RViz配置路径（这里复用了nav2路径）
    rviz_pkg = get_package_share_directory("e07_nav2_bringup")

    # -------- 加载SLAM的launch文件 --------
    # 加载SLAM的cartographer_launch.py文件（用于启动Cartographer SLAM）
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_pkg, 'launch', 'cartographer_launch.py')
        )
    )

    # -------- 加载导航的launch文件 --------
    # 加载导航系统的launch文件（一般启动地图服务器、定位器、路径规划等）
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg, 'launch', 'navigation_launch.py')
        )
    )

    # -------- 加载RViz的launch文件 --------
    # 加载RViz配置启动文件，用于可视化地图、机器人姿态、路径等信息
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rviz_pkg, 'launch', 'rviz_launch.py')
        )
    )

    # -------- 构造LaunchDescription对象并添加上述三个launch --------
    ld = LaunchDescription()
    ld.add_action(slam_launch)   # 启动SLAM
    ld.add_action(nav2_launch)   # 启动导航
    ld.add_action(rviz_launch)   # 启动RViz

    return ld  # 返回整体launch描述对象
