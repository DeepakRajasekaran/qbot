from launch import LaunchDescription
from pathlib import Path
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    qbot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('qbot_description'), '/launch/qbot_description.launch.py']
        )
    )

    gazebo_resource_path = SetEnvironmentVariable(
                            name='GZ_SIM_RESOURCE_PATH',
                            value=[
                                    str(Path(get_package_share_directory('qbot_description')).parent.resolve())
                                  ]
                            )   

    world_resource_path = str(Path(get_package_share_directory('qbot_simulator')).joinpath('worlds').resolve())

    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('ros_gz_sim'), '/launch/gz_sim.launch.py']
        ),
        launch_arguments=[
                          ("gz_args", [f"{world_resource_path}/warehouse.sdf -r"])
                        # ("gz_args", ["empty.sdf -r"])
        ]
    )

    ros_gz_sim_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='ros_gz_sim_create',
        arguments=['-topic', 'robot_description'],
        output='screen'
    )

    gazebo_bridge_config_path = get_package_share_directory('qbot_simulator') + '/config/gazebo_bridge.yaml'

    ros_gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge_parameter_bridge',
        output='screen',
        parameters=[{'config_file': gazebo_bridge_config_path}]
    )

    return LaunchDescription([
        gazebo_resource_path,
        qbot_description_launch,
        gz_sim_launch,
        ros_gz_sim_node,
        ros_gz_bridge_node
    ])