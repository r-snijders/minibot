from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():
    world = str(Path(get_package_share_directory('minibot')) / 'worlds' / 'rover.sdf')
    gz_launch = str(Path(get_package_share_directory('ros_gz_sim')) / 'launch' / 'gz_sim.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('headless_rendering', default_value='false'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gz_launch),
                                 launch_arguments={'gz_args': ['-r -s ', PythonExpression(["' --headless-rendering ' if '", LaunchConfiguration('headless_rendering'), "' == 'true' else ' '"]), world]}.items()),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             remappings=[('/cmd_vel', '/drive/cmd_vel')],
             arguments=['/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                        '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                        '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                        '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                        '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                        '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--x', '0', '--y', '0', '--z', '0.09', '--frame-id', 'base_link', '--child-frame-id', 'laser_frame']),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--x', '0.09', '--y', '0', '--z', '0.08', '--roll', '-1.5707963', '--pitch', '0', '--yaw', '-1.5707963',
                        '--frame-id', 'base_link', '--child-frame-id', 'camera_optical_frame']),
    ])
