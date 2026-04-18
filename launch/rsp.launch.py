import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro
from launch_ros.parameter_descriptions import ParameterValue



def generate_launch_description():

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    sim_mode     = LaunchConfiguration('sim_mode')

    # Process the URDF/xacro file, passing sim_mode so ros2_control.xacro
    # can select the correct hardware plugin at parse time.
    pkg_path  = os.path.join(get_package_share_directory('wall-e-robot'))
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')

    # xacro.process_file needs Python-evaluated values, not LaunchConfigurations,
    # so we read the argument from the environment / default at launch time via
    # EnvironmentVariable substitution.  A cleaner approach is to use
    # Command() substitution in the Node parameters so xacro is invoked at
    # launch time (not import time) — allows sim_mode to be set dynamically.
    from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
    from launch_ros.substitutions import FindPackageShare

    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([FindPackageShare('wall-e-robot'), 'description', 'robot.urdf.xacro']),
        ' sim_mode:=', sim_mode,
    ])

    params = {
        'robot_description': ParameterValue(robot_description_content, value_type=str),
        'use_sim_time': use_sim_time,
    }


    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        DeclareLaunchArgument(
            'sim_mode',
            default_value='false',
            description='Start in simulation mode (uses gz_ros2_control hardware plugin)'),

        node_robot_state_publisher
    ])
