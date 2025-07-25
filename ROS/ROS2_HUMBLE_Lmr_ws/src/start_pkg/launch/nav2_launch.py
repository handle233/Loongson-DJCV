import os  # ���� Python �� os ģ�飬���ڴ����ļ�·������

from launch import LaunchDescription  # LaunchDescription �� ROS2 ����ϵͳ�ĺ����࣬��������Ҫ������Щ����

from launch.actions import IncludeLaunchDescription, TimerAction  # IncludeLaunchDescription ���ڰ������� launch �ļ���TimerAction �����ӳ�ִ��ĳЩ����

from launch.launch_description_sources import PythonLaunchDescriptionSource  # ָ���������� launch �ļ��� Python ���ͣ�.py��

from ament_index_python.packages import get_package_share_directory  # ���ڻ�ȡĳ�� ROS2 ���� share Ŀ¼·��������װ·����


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
