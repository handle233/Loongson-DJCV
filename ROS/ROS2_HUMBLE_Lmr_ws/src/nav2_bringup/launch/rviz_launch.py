# -------------------- 版权信息 --------------------
# 版权所有 © 2018 Intel Corporation
#
# 根据 Apache License, Version 2.0 协议发布
# 你只能在遵守许可协议的前提下使用本文件
# 你可以在以下网址查看完整的许可证内容：
#     http://www.apache.org/licenses/LICENSE-2.0
#
# 除非适用法律要求或书面同意，否则本文件按“原样”提供，
# 不附带任何明示或暗示的担保或责任。

# -------------------- 引入所需模块 --------------------

import os  # Python 的标准库，用于处理文件路径

# ROS 2 的工具函数，用于获取某个 ROS 2 包的“share”目录路径
from ament_index_python.packages import get_package_share_directory

# ROS 2 启动系统的核心类，用于创建启动描述（LaunchDescription）
from launch import LaunchDescription

# 以下类用于处理高级启动行为，如参数声明、事件触发、条件执行等（虽然本例中没用到，但预留了）
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration

# 启动 ROS 2 节点的专用类（这是本例中最核心的功能）
from launch_ros.actions import Node

# nav2_common 包中的一个工具类，用于字符串替换（本例中未使用）
from nav2_common.launch import ReplaceString

# -------------------- 启动文件主体函数 --------------------

def generate_launch_description():
    # """
    # ROS 2 启动系统会自动调用此函数，返回一个 LaunchDescription 对象，
    # 里面包含了我们要启动的所有节点与配置。
    # """

    # ========== 1. 获取 RViz2 配置文件的绝对路径 ==========
    # 这个 RViz 配置文件位于 e07_nav2_bringup 包中的 rviz 目录下，文件名为 nav2_default_view.rviz
    # 它决定了 RViz 打开时默认加载哪些显示项（如地图、激光雷达、路径规划线等）
    rviz2_config = os.path.join(
        get_package_share_directory('e07_nav2_bringup'),  # 获取包路径
        'rviz',                                            # 子文件夹
        'nav2_default_view.rviz'                           # 配置文件名
    )

    # ========== 2. 创建一个 Node，用于启动 RViz2 ==========
    # Node 是 ROS 2 启动系统中用于启动节点的对象
    rviz2_node = Node(
        package='rviz2',                  # 指定使用哪个 ROS 2 包，这里是 rviz2 包
        executable='rviz2',              # 可执行文件名，通常与 package 名一致
        name='nav2_rviz2_show',          # 给这个节点起个名字，方便在 rqt_graph 等工具中查看
        arguments=['-d', rviz2_config],  # 启动参数，-d 表示指定配置文件路径
        output='screen'                  # 输出方式，'screen' 表示打印输出到终端
    )

    # ========== 3. 创建启动描述对象 ==========
    # LaunchDescription 是 ROS 2 启动文件的核心对象，相当于一个“启动清单”
    ld = LaunchDescription()

    # ========== 4. 将上面定义的 RViz 节点加入到启动清单中 ==========
    ld.add_action(rviz2_node)

    # ========== 5. 返回完整的启动描述 ==========
    # ROS 2 启动系统会根据这个对象实际启动节点
    return ld
