
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg = get_package_share_directory('ackermann_bot')
    urdf_file = os.path.join(pkg, 'urdf', 'ack.urdf.xacro')
    world_file = os.path.join(pkg, 'worlds', 'shapes.sdf')
    bridge_config = os.path.join(pkg, 'config', 'bridge.yaml')

    # robot_description = output of `xacro <file>` as a STRING.
    # The value_type=str is critical — without it, ROS 2 tries to
    # YAML-parse the XML and crashes.
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    # Launch Gazebo Harmonic with our world.
    # -r flag = run physics immediately (don't wait for play button).
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': '-r ' + world_file}.items()
    )

    # robot_state_publisher: URDF -> TF tree + /robot_description topic.
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    # Spawn the robot in Gazebo by reading the /robot_description topic.
    # Spawn pose: x=0, y=0, z=0.05 (just above ground so wheels touch).
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'ackermann_bot',
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '0.05'
        ],
        output='screen'
    )

    # parameter_bridge — translate topics per bridge.yaml
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_config,
            'qos_overrides./tf_static.publisher.durability': 'transient_local'
        }],
        output='screen'
    )

    return LaunchDescription([gz_sim, rsp, spawn, bridge])
