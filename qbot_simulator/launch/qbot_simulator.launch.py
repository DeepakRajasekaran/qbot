from launch import LaunchDescription
from pathlib import Path
import os

from launch_ros.actions import Node
from launch.actions import (
    IncludeLaunchDescription,
    AppendEnvironmentVariable,
    TimerAction,
    LogInfo
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError


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
    # Gazebo resource path
    # -------------------------------
    qbot_sim_pkg = get_package_share_directory('qbot_simulator')
    qbot_desc_pkg = get_package_share_directory('qbot_description')
    qbot_desc_model_path = str(Path(qbot_desc_pkg).parent.resolve())

    gazebo_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(qbot_sim_pkg, 'models') + os.pathsep + qbot_desc_model_path
    )

    # Add turtlebot3_gazebo models to the resource path if available
    try:
        tb3_models_path = os.path.join(
            get_package_share_directory('turtlebot3_gazebo'), 'models')
        tb3_resource_path = AppendEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=tb3_models_path
        )
    except PackageNotFoundError:
        tb3_resource_path = None

    # -------------------------------
    # Gazebo world
    # -------------------------------
    world_path = str(
        Path(qbot_sim_pkg)
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

    # -------------------------------
    # 3D Lidar Frame Fix
    # -------------------------------
    # Bridge the Gazebo-generated frame to the ROS URDF frame
    lidar_tf_fix = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_tf_fix',
        arguments=['0', '0', '0', '0', '0', '0', 'lidar', 'qbot/base_footprint/lidar']
    )

    ld = LaunchDescription([
        gazebo_resource_path,
        qbot_description_launch,
        gz_sim_launch,
        gz_sim_gui,
        delayed_spawn,
        ros_gz_bridge_node,
        scan_frame_republisher_node,
        lidar_tf_fix
    ])

    if tb3_resource_path:
        ld.add_action(tb3_resource_path)
    else:
        ld.add_action(LogInfo(msg='WARNING: turtlebot3_gazebo not found. "turtlebot3_world" model may be missing.'))

    return ld
