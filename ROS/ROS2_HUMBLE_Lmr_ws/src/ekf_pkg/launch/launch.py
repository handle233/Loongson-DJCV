import os  
import launch_ros
import launch

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='ekf_pkg').find('ekf_pkg')

    robot_localization_node = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'ekf.yaml'), {'use_sim_time': False}]
    )

    return launch.LaunchDescription([
        robot_localization_node,
    ])

