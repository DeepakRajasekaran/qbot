from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # --------------------------------------------------
    # Launch arguments
    # --------------------------------------------------
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz'
    )

    # --------------------------------------------------
    # Package paths
    # --------------------------------------------------
    pkg_qbot_slam = get_package_share_directory('qbot_slam')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    slam_params = os.path.join(
        pkg_qbot_slam, 'config', 'slam_toolbox.yaml'
    )

    rviz_config = os.path.join(
        pkg_qbot_slam, 'rviz', 'slam.rviz'
    )

    slam_toolbox_launch = os.path.join(
        pkg_slam_toolbox, 'launch', 'online_async_launch.py'
    )

    # --------------------------------------------------
    # Include slam_toolbox launch
    # --------------------------------------------------
    slam_toolbox_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_launch),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params
        }.items()
    )

    # --------------------------------------------------
    # RViz node
    # --------------------------------------------------
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # --------------------------------------------------
    # Launch description
    # --------------------------------------------------
    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_use_rviz)

    ld.add_action(slam_toolbox_include)
    ld.add_action(rviz_node)

    return ld
