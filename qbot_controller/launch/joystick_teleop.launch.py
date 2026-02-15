import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    joy_dev = LaunchConfiguration('joy_dev')
    serial_port = LaunchConfiguration('serial_port')
    baud_rate = LaunchConfiguration('baud_rate')

    joy_config = os.path.join(
        get_package_share_directory('qbot_controller'),
        'config',
        'joystick.yaml'
    )

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_joy_dev = DeclareLaunchArgument(
        'joy_dev',
        default_value='/dev/input/js0',
        description='Device file for the joystick'
    )

    declare_serial_port = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for the robot microcontroller'
    )

    declare_baud_rate = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Baud rate for serial communication'
    )

    # Joy Node - Reads joystick input
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': joy_dev,
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
            'use_sim_time': use_sim_time
        }]
    )

    # Teleop Twist Joy Node - Converts joystick events to velocity commands
    # Default config for Xbox-style controller:
    #   Enable Button: RB (Button 5) - Must be held to move
    #   Linear Axis: Left Stick Vertical (Axis 1)
    #   Angular Axis: Right Stick Horizontal (Axis 3)
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[
            joy_config,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/cmd_vel', '/cmd_vel')
        ]
    )

    # Serial Bridge Node - Communicates with the robot firmware
    serial_bridge_node = Node(
        package='qbot_controller',
        executable='serial_bridge',
        name='serial_bridge_node',
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            'baud_rate': baud_rate,
            'use_sim_time': use_sim_time
        }]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_joy_dev,
        declare_serial_port,
        declare_baud_rate,
        joy_node,
        teleop_node,
        serial_bridge_node
    ])