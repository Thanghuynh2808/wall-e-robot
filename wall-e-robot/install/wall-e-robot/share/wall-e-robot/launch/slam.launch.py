import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'wall-e-robot'

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Đường dẫn tới file config vừa tạo
    slam_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'mapper_params_online_async.yaml')

    # Khai báo node slam_toolbox
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        slam_toolbox_node
    ])
