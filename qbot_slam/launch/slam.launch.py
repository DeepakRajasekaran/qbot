from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():

    # --------------------------------------------------
    # Launch arguments
    # --------------------------------------------------
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')

    # Read default mode from config file
    pkg_qbot_slam = get_package_share_directory('qbot_slam')
    config_file_path = os.path.join(pkg_qbot_slam, 'config', 'slam_config.yaml')
    default_slam_mode = '2d'
    default_slam_method = 'depth' # Options: 'depth', 'scan_cloud'

    if os.path.exists(config_file_path):
        try:
            with open(config_file_path, 'r') as f:
                config_data = yaml.safe_load(f)
                if config_data and 'slam_mode' in config_data:
                    default_slam_mode = config_data['slam_mode']
                if config_data and 'slam_method' in config_data:
                    default_slam_method = config_data['slam_method']
        except Exception as e:
            print(f"Warning: Could not read slam_config.yaml: {e}")

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

    slam_mode = LaunchConfiguration('slam_mode')
    declare_slam_mode = DeclareLaunchArgument(
        'slam_mode',
        default_value=default_slam_mode,
        description='SLAM mode: "2d" or "3d"'
    )

    slam_method = LaunchConfiguration('slam_method')
    declare_slam_method = DeclareLaunchArgument(
        'slam_method',
        default_value=default_slam_method,
        description='3D SLAM Method: "depth" (RGB-D) or "scan_cloud" (3D Lidar)'
    )

    delete_db = LaunchConfiguration('delete_db')
    declare_delete_db = DeclareLaunchArgument(
        'delete_db',
        default_value='true',
        description='Delete RTAB-Map database on start (only for 3d mode)'
    )

    map_id = LaunchConfiguration('map_id')
    declare_map_id = DeclareLaunchArgument(
        'map_id',
        default_value='my_map',
        description='Map ID for saving 3D map (e.g. map_01)'
    )

    # --------------------------------------------------
    # Package paths
    # --------------------------------------------------
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    slam_params = os.path.join(
        pkg_qbot_slam, 'config', 'slam_toolbox.yaml'
    )

    rviz_config = PythonExpression([
        "'", pkg_qbot_slam, "/rviz/slam_' + '", slam_mode, "' + '.rviz'"
    ])

    slam_toolbox_launch = os.path.join(
        pkg_slam_toolbox, 'launch', 'online_async_launch.py'
    )

    # --------------------------------------------------
    # 2D SLAM: slam_toolbox
    # --------------------------------------------------
    slam_toolbox_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_launch),
        condition=IfCondition(PythonExpression(["'", slam_mode, "' == '2d'"])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params
        }.items()
    )

    # --------------------------------------------------
    # 3D SLAM: RTAB-Map
    # --------------------------------------------------
    # Define workspace maps directory
    workspace_path = os.path.expanduser('~/jazzy_ws/qbot_ws')
    maps_dir = os.path.join(workspace_path, 'maps', '3d')
    
    if not os.path.exists(maps_dir):
        os.makedirs(maps_dir)

    rtabmap_node = Node(
        condition=IfCondition(PythonExpression(["'", slam_mode, "' == '3d'"])),
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'frame_id': 'base_footprint',
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'subscribe_depth': PythonExpression(["'", slam_method, "' in ['depth', 'fused']"]),
            'subscribe_rgb': PythonExpression(["'", slam_method, "' in ['depth', 'fused']"]),
            'subscribe_scan': False,
            'subscribe_scan_cloud': True,
            'subscribe_scan_cloud': PythonExpression(["'", slam_method, "' in ['scan_cloud', 'fused']"]),
            'approx_sync': True,
            'use_sim_time': use_sim_time,
            'queue_size': 20,
            # RTAB-Map Internal Parameters
            'Mem/IncrementalMemory': 'true',
            'Mem/InitWMWithAllNodes': 'true'
        }],
        remappings=[
            ('scan_cloud', '/lidar/points'),
            ('rgb/image', '/rtabmap/rgb/image'),
            ('rgb/camera_info', '/rtabmap/rgb/camera_info'),
            ('depth/image', '/rtabmap/depth/image'),
            ('odom', '/odom'),
            ('grid_map', '/map'),
        ],
        arguments=[PythonExpression(["'--delete_db_on_start' if '", delete_db, "' == 'true' else ''"])]
    )

    # --------------------------------------------------
    # Save Map on Exit (3D Mode)
    # --------------------------------------------------
    save_map_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=rtabmap_node,
            on_exit=[
                LogInfo(msg='RTAB-Map process exited. Moving database to maps folder...'),
                ExecuteProcess(
                    cmd=[
                        'mkdir -p ', maps_dir, ' && ',
                        'test -f ', os.path.expanduser('~/.ros/rtabmap.db'), ' && cp ', os.path.expanduser('~/.ros/rtabmap.db'), ' ', 
                        PythonExpression(["'", maps_dir, "/' + '", map_id, "' + '.db'"])
                    ],
                    shell=True
                ),
                LogInfo(msg=['Map saved to: ', maps_dir, '/', map_id, '.db'])
            ]
        ),
        condition=IfCondition(PythonExpression(["'", slam_mode, "' == '3d'"]))
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
    ld.add_action(declare_slam_mode)
    ld.add_action(declare_slam_method)
    ld.add_action(declare_delete_db)
    ld.add_action(declare_map_id)

    ld.add_action(slam_toolbox_include)
    ld.add_action(rtabmap_node)
    ld.add_action(save_map_handler)
    ld.add_action(rviz_node)

    return ld
