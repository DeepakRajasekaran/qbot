from launch import LaunchDescription
from pathlib import Path

from launch_ros.actions import Node
from launch.actions import (
    IncludeLaunchDescription,
    AppendEnvironmentVariable,
    TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # -------------------------------
    # Robot description
    # -------------------------------
    qbot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                get_package_share_directory('qbot_description'),
                '/launch/qbot_description.launch.py'
            ]
        )
    )

    # -------------------------------
    # Gazebo resource path (PATCHED)
    # -------------------------------
    gazebo_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=str(
            Path(get_package_share_directory('qbot_description')).parent.resolve()
        )
    )

    # -------------------------------
    # Gazebo world
    # -------------------------------
    world_path = str(
        Path(get_package_share_directory('qbot_simulator'))
        .joinpath('worlds', 'turtlebot3_world.world')
        .resolve()
    )

    gz_sim_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    get_package_share_directory('ros_gz_sim'),
                    '/launch/gz_sim.launch.py'
                ]
            ),
            launch_arguments={
                'gz_args': f'-r -s {world_path}'
            }.items()
        )

    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                get_package_share_directory('ros_gz_sim'),
                '/launch/gz_sim.launch.py'
            ]
        ),
        launch_arguments={
            'gz_args': '-g'
        }.items()
    )


    # -------------------------------
    # Spawn robot (DELAYED)
    # -------------------------------
    spawn_qbot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '--name', 'qbot',
            '--x', '-2.0',
            '--y', '0.0'
        ],
        output='screen'
    )

    delayed_spawn = TimerAction(
        period=5.0,
        actions=[spawn_qbot]
    )

    # -------------------------------
    # Gazebo ↔ ROS bridge
    # -------------------------------
    ros_gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file':
            get_package_share_directory('qbot_simulator') +
            '/config/gazebo_bridge.yaml'
        }]
    )

    scan_frame_republisher_node = Node(
        package='qbot_simulator',
        executable='sensor_frame_fix',
        name='sensor_frame_fix',
        parameters=[{'scan_frame_id': 'lidar'}, 
                    {'imu_frame_id': 'imu'}],
    )


    return LaunchDescription([
        gazebo_resource_path,
        qbot_description_launch,
        gz_sim_launch,
        gz_sim_gui,
        delayed_spawn,
        ros_gz_bridge_node,
        scan_frame_republisher_node
    ])
