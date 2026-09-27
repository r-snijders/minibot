from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():
    world = str(Path(get_package_share_directory('minibot')) / 'worlds' / 'rover.sdf')
    gz_launch = str(Path(get_package_share_directory('ros_gz_sim')) / 'launch' / 'gz_sim.launch.py')
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gz_launch),
                                 launch_arguments={'gz_args': '-r ' + world}.items()),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             arguments=['/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                        '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']),
    ])
