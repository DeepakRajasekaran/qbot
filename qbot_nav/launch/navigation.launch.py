from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    use_rviz = LaunchConfiguration('use_rviz')

    pkg_qbot_nav = get_package_share_directory('qbot_nav')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    nav2_params = os.path.join(pkg_qbot_nav, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(pkg_qbot_nav, 'rviz', 'nav2.rviz')

    nav2_launch = os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml,
            # 'params_file': nav2_params,
            'autostart': 'true'
        }.items()
    )

    rviz = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true'))
    ld.add_action(DeclareLaunchArgument('use_rviz', default_value='true'))
    ld.add_action(DeclareLaunchArgument('map'))
    ld.add_action(nav2)
    ld.add_action(rviz)

    return ld
