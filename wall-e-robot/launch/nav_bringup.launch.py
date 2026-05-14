import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def generate_launch_description():

    package_name = 'wall-e-robot'

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    # Lấy đường dẫn các file cần thiết
    nav2_bringup_dir = get_package_share_directory(package_name)
    
    # Sử dụng file bringup chuẩn của Nav2 để quản lý tập trung toàn bộ các node
    nav2_main_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    # Bọc toàn bộ vào GroupAction để Remap topic cmd_vel cho tất cả các node navigation
    bringup_with_remap = GroupAction(
        actions=[
            SetRemap(src='/cmd_vel', dst='/diff_drive_controller/cmd_vel_unstamped'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav2_main_launch_dir, 'bringup_launch.py')),
                launch_arguments={
                    'map': map_yaml_file,
                    'use_sim_time': use_sim_time,
                    'params_file': params_file,
                    'autostart': 'true'
                }.items(),
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(nav2_bringup_dir, 'maps', 'my_map.yaml'),
            description='Full path to map yaml file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(nav2_bringup_dir, 'config', 'nav2_params.yaml'),
            description='Full path to the ROS2 parameters file to use'),

        bringup_with_remap
    ])
