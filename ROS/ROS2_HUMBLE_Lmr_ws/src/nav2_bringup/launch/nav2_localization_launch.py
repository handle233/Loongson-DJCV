import os  # 引入 Python 的 os 模块，用于处理文件路径操作

from launch import LaunchDescription  # LaunchDescription 是 ROS2 启动系统的核心类，用于描述要启动哪些动作

from launch.actions import IncludeLaunchDescription, TimerAction  # IncludeLaunchDescription 用于包含其他 launch 文件；TimerAction 用于延迟执行某些启动

from launch.launch_description_sources import PythonLaunchDescriptionSource  # 指定被包含的 launch 文件是 Python 类型（.py）

from ament_index_python.packages import get_package_share_directory  # 用于获取某个 ROS2 包的 share 目录路径（即安装路径）

def generate_launch_description():
    # 获取“e07_nav2_bringup”包的 share 路径，也就是该包中包含 launch 文件的路径
    pkg_share = get_package_share_directory("e07_nav2_bringup")

    # 启动 RViz2 可视化界面，便于我们实时查看地图、机器人位置、路径等
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'rviz_launch.py')  # 拼接出 rviz 启动文件的完整路径
        )
    )

    # 启动定位功能（如 AMCL 定位），用于确定机器人在地图上的当前位置
    loca_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'local_launch.py')  # 拼接出定位系统的启动文件路径
        )
    )

    # 启动导航功能（如路径规划、控制器），用于让机器人根据目标点规划路径并运动
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'nav2_dwb_launch.py')  # 拼接出导航系统的启动文件路径
        )
    )

    # 使用 TimerAction 实现延迟启动导航系统，设置延迟时间为 1 秒，防止其抢先启动
    delayed_nav2_launch = TimerAction(
        period=1.0,  # 延迟时间为 1.0 秒
        actions=[nav2_launch]  # 1 秒后执行 nav2_launch 启动动作
    )

    # 使用 TimerAction 实现延迟启动定位系统，设置延迟时间为 2 秒，确保 RViz 启动完成
    delayed_loca_launch = TimerAction(
        period=2.0,  # 延迟时间为 2.0 秒
        actions=[loca_launch]  # 2 秒后执行定位系统启动
    )

    # 将所有启动动作封装在一个 LaunchDescription 中，ROS2 启动系统会依次执行这些动作
    return LaunchDescription([
        rviz_launch,            # 最先启动 RViz（可视化工具）
        delayed_loca_launch,    # 延迟 2 秒启动定位
        delayed_nav2_launch     # 延迟 1 秒启动导航
    ])
