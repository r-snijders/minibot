from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0'),
        Node(package='minibot', executable='hardware', output='screen',
             remappings=[('/cmd_vel', '/drive/cmd_vel')],
             parameters=[{'port': LaunchConfiguration('port')}]),
        Node(package='minibot', executable='velocity_gate', output='screen'),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--x', '0', '--y', '0', '--z', '0.09', '--frame-id', 'base_link', '--child-frame-id', 'laser_frame']),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--x', '0.09', '--y', '0', '--z', '0.08', '--roll', '-1.5707963', '--pitch', '0', '--yaw', '-1.5707963',
                        '--frame-id', 'base_link', '--child-frame-id', 'camera_optical_frame']),
    ])
