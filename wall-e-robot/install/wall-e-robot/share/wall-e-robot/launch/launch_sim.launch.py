import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'wall-e-robot'

    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([
            FindPackageShare(package_name),
            'worlds',
            'empty.world' # Hoặc đổi thành 'office.sdf' làm mặc định nếu bạn muốn
        ]),
        description='World to load'
    )

    # ── Robot State Publisher (sim_mode=true forces gz_ros2_control plugin) ──
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(package_name), 'launch', 'rsp.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'sim_mode':     'true',
            'enable_sensors': 'false',
        }.items()
    )

    # ── Gazebo Sim ──
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={
            # SSH/headless: chạy server-only để không cần X11/GUI
            'gz_args': [PathJoinSubstitution([FindPackageShare(package_name), 'worlds', world]), ' -r -s -v4 '],
            'on_exit_shutdown':  'true',
        }.items()

    )

    # ── Spawn robot entity into Gazebo ──
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name',  'my_bot',
            '-z',     '0.1',
        ],
        output='screen'
    )

    # ── ROS-Gazebo bridge (clock, scan, camera, tf, odom …) ──
    bridge_params = os.path.join(
        get_package_share_directory(package_name), 'config', 'gz_bridge.yaml'
    )
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p', f'config_file:={bridge_params}',
        ]
    )

    # ── ros2_control: spawn joint_state_broadcaster ──
    #    Chờ spawn_entity hoàn tất trước khi spawn controllers
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
        ],
    )

    # ── ros2_control: spawn diff_drive_controller ──
    #    Chờ joint_state_broadcaster sẵn sàng trước
    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--controller-manager', '/controller_manager',
        ],
    )

    # Spawn diff_drive_controller chỉ sau khi joint_state_broadcaster đã inactive/active
    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner],
        )
    )

    return LaunchDescription([
        world_arg,
        rsp,
        gazebo,
        spawn_entity,
        ros_gz_bridge,
        joint_state_broadcaster_spawner,
        delayed_diff_drive_spawner,
    ])
