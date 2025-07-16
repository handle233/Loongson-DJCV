# 引入Python标准库os，用于处理文件路径等操作
import os

# 从ament_index_python中导入函数，用于获取ROS2中某个包的共享目录（share文件夹路径）
from ament_index_python.packages import get_package_share_directory

# 从launch库中导入LaunchDescription类，这是ROS2中用于定义启动描述的类
from launch import LaunchDescription
# 从launch.actions中导入IncludeLaunchDescription类，用于在这个launch文件中包含其他的launch文件
from launch.actions import IncludeLaunchDescription
# 从launch.launch_description_sources中导入PythonLaunchDescriptionSource，用于指定被包含的launch文件是Python格式的
from launch.launch_description_sources import PythonLaunchDescriptionSource

# 定义一个函数 generate_launch_description()，这是ROS2启动系统会自动调用的函数，用于返回要执行的启动描述
def generate_launch_description():

    # 获取“e05_slam_toolbox_mapping”这个包的share目录，用于找到里面的launch文件
    slam_pkg = get_package_share_directory("e05_slam_toolbox_mapping")

    # 获取“e07_nav2_bringup”包的share目录（这里用于导航和rviz两个用途）
    nav2_pkg = get_package_share_directory("e07_nav2_bringup")
    rviz_pkg = get_package_share_directory("e07_nav2_bringup")

    # 创建slam_launch，用于启动SLAM模块（同时建图与定位）
    # 它包含的是slam_pkg包下的launch/slam_toolbox_launch.py文件
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_pkg, 'launch', 'slam_toolbox_launch.py')
        )
    )

    # 创建nav2_launch，用于启动导航模块（例如路径规划、控制器等）
    # 它包含的是nav2_pkg包下的launch/navigation_launch.py文件
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg, 'launch', 'navigation_launch.py')
        )
    )

    # 创建rviz_launch，用于启动可视化工具RViz2，用于观察SLAM和导航的运行状态
    # 它包含的是rviz_pkg包下的launch/rviz_launch.py文件
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rviz_pkg, 'launch', 'rviz_launch.py')
        )
    )

    # 创建一个LaunchDescription对象，这个对象是所有启动动作的容器
    ld = LaunchDescription()

    # 将上面定义的三个启动动作依次添加到LaunchDescription中
    ld.add_action(slam_launch)
    ld.add_action(nav2_launch)
    ld.add_action(rviz_launch)

    # 返回这个启动描述，ROS2的launch系统会据此启动所有子系统
    return ld
