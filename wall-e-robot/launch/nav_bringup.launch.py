import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'wall-e-robot'

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    # 1. Đường dẫn các file
    nav2_bringup_dir = get_package_share_directory(package_name)
    nav2_launch_dir = os.path.join(nav2_bringup_dir, 'launch')

    # 2. Khai báo các Action
    # Map Server
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'yaml_filename': map_yaml_file}]
    )

    # AMCL (Định vị)
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    # Navigation (Planner, Controller, v.v.)
    navigation_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'navigation_launch.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time,
                          'params_file': params_file,
                          'autostart': 'true'}.items()
    )

    # Lifecycle Manager để kích hoạt Map Server và AMCL
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ['map_server', 'amcl']}]
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

        map_server_node,
        amcl_node,
        navigation_nodes,
        lifecycle_manager_node
    ])
