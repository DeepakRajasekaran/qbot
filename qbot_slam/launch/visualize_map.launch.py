from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    TimerAction,
    ExecuteProcess,
    IncludeLaunchDescription
)
from launch_ros.actions import Node
from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # --------------------------------------------------
    # Arguments
    # --------------------------------------------------
    map_id = LaunchConfiguration('map_id')
    declare_map_id = DeclareLaunchArgument(
        'map_id',
        default_value='my_map',
        description='Map ID to visualize (e.g. map_01)'
    )

    map_path_arg = LaunchConfiguration('map_path')
    declare_map_path = DeclareLaunchArgument(
        'map_path',
        default_value='',
        description='Full path to map database file. If provided, overrides map_id.'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock (set true if running with Gazebo)'
    )

    # --------------------------------------------------
    # Paths
    # --------------------------------------------------
    pkg_qbot_slam = get_package_share_directory('qbot_slam')
    rviz_config = os.path.join(pkg_qbot_slam, 'rviz', 'slam_3d.rviz')

    # Path to the saved map database — resolves to: ~/jazzy_ws/qbot_ws/maps/3d/<map_id>.db
    maps_dir = os.path.join(os.path.expanduser('~/jazzy_ws/qbot_ws'), 'maps', '3d')
    
    map_path = PythonExpression([
        "'", map_path_arg, "' if '", map_path_arg, "' != '' else '", maps_dir, "/' + '", map_id, "' + '.db'"
    ])

    # --------------------------------------------------
    # Robot Description (Required for Sensor Transforms)
    # --------------------------------------------------
    qbot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('qbot_description'),
            '/launch/qbot_description.launch.py'
        ])
    )

    # --------------------------------------------------
    # RTAB-Map Node (Visualization / Localization Mode)
    #
    # Key parameters:
    #   Mem/IncrementalMemory  = false  → do NOT add new nodes to the map
    #   Mem/InitWMWithAllNodes = true   → load ALL nodes from DB into working memory
    #   Rtabmap/PublishMapPerUpdate     → publish map immediately on load
    #   RGBD/Enabled           = false  → no live camera input expected
    #   subscribe_*            = false  → no sensor subscriptions
    # --------------------------------------------------
    rtabmap_viz_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            # --- Database ---
            'database_path':              map_path,

            # --- Frames ---
            'frame_id':                   'base_footprint', # Match slam.launch.py
            'map_frame_id':               'map',         # map frame
            'odom_frame_id':              'odom',        # odometry frame

            # --- Clock ---
            'use_sim_time':               use_sim_time,

            # --- Disable all live sensor subscriptions ---
            'subscribe_depth':            False,
            'subscribe_rgb':              False,
            'subscribe_scan':             False,
            'subscribe_scan_cloud':       False,
            'subscribe_odom_info':        False,
            'subscribe_odom':             False,
            'subscribe_stereo':           False,
            'subscribe_rgbd':             False,

            # --- TF publishing ---
            'publish_tf':                 False,         # Disable to avoid conflict with static TF
            'tf_delay':                   0.05,

            # --- Visualization-only mode (read-only from DB) ---
            'Mem/IncrementalMemory':      'false',       # freeze the map
            'Mem/InitWMWithAllNodes':     'true',        # load entire DB on start

            # --- Disable depth-based grid (not needed for viz) ---
            'Grid/FromDepth':             'false',

            # --- Enable RGBD processing for graph optimization ---
            'RGBD/Enabled':               'true',

            # --- Publish map immediately when a node is updated ---
            'Rtabmap/PublishMapPerUpdate': 'true',

            # --- Sync (irrelevant when not subscribing, but keep false) ---
            'approx_sync':                False,
        }],
        remappings=[
            # 2D occupancy grid  → standard /map topic
            ('grid_map',   '/map'),
            # 3D point cloud map → /cloud_map (add PointCloud2 in RViz2)
            ('cloud_map',  '/cloud_map'),
            # Pose graph markers → /rtabmap/mapGraph (add MarkerArray in RViz2)
            ('mapGraph',   '/rtabmap/mapGraph'),
        ]
    )

    # --------------------------------------------------
    # Static TF publishers
    #
    # world → map   : needed so RViz2 can resolve the 'map' fixed frame
    # map   → odom  : dummy odom when no robot odometry is running
    # odom  → base_link : dummy base when no robot is running
    # --------------------------------------------------
    static_tf_world_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_world_map',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--yaw', '0', '--pitch', '0', '--roll', '0',
            '--frame-id', 'world',
            '--child-frame-id', 'map'
        ]
    )

    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--yaw', '0', '--pitch', '0', '--roll', '0',
            '--frame-id', 'map',
            '--child-frame-id', 'odom'
        ]
    )

    static_tf_odom_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_footprint',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--yaw', '0', '--pitch', '0', '--roll', '0',
            '--frame-id', 'odom',
            '--child-frame-id', 'base_footprint'
        ]
    )

    # --------------------------------------------------
    # Auto-trigger map publishing after rtabmap has started
    #
    # RTAB-Map may not push the map automatically on load.
    # We call /rtabmap/publish_map after a 5-second delay
    # to ensure the node is fully initialised before the call.
    # --------------------------------------------------
    trigger_publish_map = TimerAction(
        period=5.0,   # seconds — increase if your DB is very large
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2 service call /rtabmap/publish_map rtabmap_msgs/srv/PublishMap "{global_map: true, optimized: true, graph_only: false}"'
                ],
                shell=True,
                output='screen'
            )
        ]
    )

    # --------------------------------------------------
    # RViz2
    # --------------------------------------------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # --------------------------------------------------
    # Launch Description
    # --------------------------------------------------
    return LaunchDescription([
        declare_map_id,
        declare_map_path,
        declare_use_sim_time,

        LogInfo(msg=['[visualize_map] Loading DB from: ', map_path]),
        LogInfo(msg=['[visualize_map] RViz config   : ' + rviz_config]),

        # 1. Static TF tree (world → map → odom → base_footprint) & Robot State
        static_tf_world_map,
        static_tf_map_odom,
        static_tf_odom_footprint,
        qbot_description_launch,

        # 2. RTAB-Map in read-only / visualization mode
        rtabmap_viz_node,

        # 3. Trigger map publish after node is ready
        trigger_publish_map,

        # 4. RViz2 for visualization
        rviz_node,
    ])