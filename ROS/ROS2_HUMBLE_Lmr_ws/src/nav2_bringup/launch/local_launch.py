# 导入必要的Python模块和ROS 2 Launch模块
import os  # Python标准库，用于路径操作

from ament_index_python.packages import get_package_share_directory  # 获取ROS 2包的share路径

from launch import LaunchDescription  # 启动描述对象，是launch文件的核心

from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable  # 参数声明、分组动作、设置环境变量
from launch.conditions import IfCondition  # 条件判断，用于是否执行某些动作

from launch.substitutions import LaunchConfiguration, PythonExpression  # 获取参数值、Python表达式处理

from launch_ros.actions import LoadComposableNodes, Node  # 启动节点（普通节点和组合节点）
from launch_ros.descriptions import ComposableNode, ParameterFile  # 组合节点描述和参数文件类

from nav2_common.launch import RewrittenYaml  # 用于在launch时动态修改yaml配置

def generate_launch_description():

    # 获取 e07_nav2_bringup 包的路径，后面加载参数文件和地图文件需要用到
    nav2_dir = get_package_share_directory('e07_nav2_bringup')

    # 创建LaunchConfiguration对象，允许用户在launch命令行传参
    namespace = LaunchConfiguration('namespace')                 # 命名空间
    map_yaml_file = LaunchConfiguration('map')                   # 地图 YAML 文件路径
    use_sim_time = LaunchConfiguration('use_sim_time')           # 是否使用仿真时间（如Gazebo）
    autostart = LaunchConfiguration('autostart')                 # 是否自动启动生命周期管理器
    params_file = LaunchConfiguration('params_file')             # 通用参数文件路径（不在此launch中使用，但可外部传入）
    use_composition = LaunchConfiguration('use_composition')     # 是否使用组合节点方式
    container_name = LaunchConfiguration('container_name')       # 组合节点容器名称
    container_name_full = (namespace, '/', container_name)       # 组合节点容器的完整路径
    use_respawn = LaunchConfiguration('use_respawn')             # 节点崩溃后是否自动重启
    log_level = LaunchConfiguration('log_level')                 # 设置日志等级（如info、warn等）

    # 生命周期管理的节点名称，供 lifecycle_manager 管理使用
    lifecycle_nodes = ['map_server', 'amcl']

    # 设置 tf 话题重映射，防止 tf 数据冲突
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # -------------- 配置地图服务器参数 -----------------
    map_param_substitutions = {
        'yaml_filename': map_yaml_file,
        'use_sim_time': use_sim_time
    }

    map_params_file = os.path.join(nav2_dir, 'params', 'map_server.yaml')  # 原始地图参数文件路径

    map_configured_params = ParameterFile(  # 使用RewrittenYaml动态重写参数值
        RewrittenYaml(
            source_file=map_params_file,
            root_key=namespace,
            param_rewrites=map_param_substitutions,
            convert_types=True),
        allow_substs=True
    )

    # -------------- 配置AMCL定位器参数 -----------------
    amcl_param_substitutions = {
        'use_sim_time': use_sim_time
    }

    amcl_params_file = os.path.join(nav2_dir, 'params', 'amcl_params.yaml')  # 原始AMCL参数文件路径

    amcl_configured_params = ParameterFile(
        RewrittenYaml(
            source_file=amcl_params_file,
            root_key=namespace,
            param_rewrites=amcl_param_substitutions,
            convert_types=True),
        allow_substs=True
    )

    # 设置环境变量，使日志输出及时刷新，不被缓冲
    stdout_linebuf_envvar = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    # -------------- 声明所有支持的命令行参数 ---------------
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='顶层命名空间')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(nav2_dir, "maps", 'ros_map.yaml'),  # 默认地图路径
        description='地图yaml文件的完整路径')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='是否使用仿真时间（如Gazebo）')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=amcl_params_file,
        description='所有节点使用的ROS2参数文件路径')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true', description='是否自动启动Nav2功能包')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='False', description='是否使用组合节点（Composable Nodes）')

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name', default_value='nav2_container', description='组合节点容器的名称')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False', description='节点崩溃后是否自动重启（不适用于组合节点）')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='warn', description='日志输出等级')

    # -------------- 普通（非组合）节点的启动方式 ---------------
    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),  # 如果未启用组合节点才执行
        actions=[
            Node(  # 地图服务器节点
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[map_configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(  # AMCL 定位节点
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[amcl_configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(  # 生命周期管理器节点
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'node_names': lifecycle_nodes
                }])
        ]
    )

    # -------------- 使用组合节点（Composable Node）的情况 ---------------
    load_composable_nodes = LoadComposableNodes(
        condition=IfCondition(use_composition),  # 仅当 use_composition 为 True 才执行
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(  # 组合式地图服务器
                package='nav2_map_server',
                plugin='nav2_map_server::MapServer',
                name='map_server',
                parameters=[map_configured_params],
                remappings=remappings),
            ComposableNode(  # 组合式 AMCL
                package='nav2_amcl',
                plugin='nav2_amcl::AmclNode',
                name='amcl',
                parameters=[amcl_configured_params],
                remappings=remappings),
            ComposableNode(  # 组合式生命周期管理器
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_localization',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'node_names': lifecycle_nodes
                }]),
        ]
    )

    # -------------- 构造最终的 LaunchDescription 返回给launch系统 ---------------
    ld = LaunchDescription()

    # 添加设置环境变量的操作
    ld.add_action(stdout_linebuf_envvar)

    # 添加参数声明指令
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    # 添加节点启动指令（根据是否使用组合节点分别执行）
    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)

    # 返回最终的启动描述对象，ROS 2 会据此启动所有节点
    return ld
