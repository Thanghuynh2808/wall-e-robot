import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node

def generate_launch_description():

    # Tên package của bạn
    package_name='wall-e-robot'

    # 1. Bao gồm file rsp.launch.py để khởi động robot_state_publisher
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'false', 'sim_mode': 'false'}.items()
    )

    # 2. Lấy đường dẫn file cấu hình controller
    controller_params_file = os.path.join(get_package_share_directory(package_name),'config','ros2_controllers.yaml')

    # 3. Lấy nội dung robot_description (URDF)
    robot_description_content = Command([
        'xacro ',
        os.path.join(get_package_share_directory(package_name), 'description', 'robot.urdf.xacro'),
        ' sim_mode:=false'
    ])

    from launch_ros.parameter_descriptions import ParameterValue

    # 4. Khởi động controller_manager (ros2_control_node)
    # Node này sẽ thực sự kết nối với STM32 qua Serial
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': ParameterValue(robot_description_content, value_type=str)}, controller_params_file],
        remappings=[
            ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
        ],
        output="screen",
    )

    # Đợi một chút để robot_state_publisher sẵn sàng rồi mới chạy controller_manager
    delayed_controller_manager = TimerAction(period=2.0, actions=[controller_manager])

    # 4. Spawner cho diff_drive_controller
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"],
    )

    # Chạy spawner sau khi controller_manager khởi động xong
    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    # 5. Spawner cho joint_state_broadcaster
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    return LaunchDescription([
        rsp,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner
    ])
