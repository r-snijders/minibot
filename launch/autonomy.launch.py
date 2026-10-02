from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('minibot'))
    slam = Path(get_package_share_directory('slam_toolbox'))
    simulation = LaunchConfiguration('simulation')
    simulated = ParameterValue(simulation, value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument('simulation', default_value='true'),
        DeclareLaunchArgument('initial_soc', default_value='0.8'),
        DeclareLaunchArgument('vision', default_value='true'),
        DeclareLaunchArgument('speech', default_value='true'),
        DeclareLaunchArgument('start_simulation', default_value='true'),
        DeclareLaunchArgument('restore_dock', default_value='false'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/sim.launch.py')),
                                 condition=IfCondition(PythonExpression(["'", simulation, "' == 'true' and '", LaunchConfiguration('start_simulation'), "' == 'true'"])),
                                 launch_arguments={'initial_soc': LaunchConfiguration('initial_soc')}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(slam/'launch/online_async_launch.py')),
                                 launch_arguments={'use_sim_time': simulation,
                                                   'slam_params_file': str(share/'config/slam.yaml')}.items()),
        Node(package='minibot', executable='autonomy', output='screen',
             parameters=[{'use_sim_time': simulated, 'simulation': simulated,
                          'restore_dock': ParameterValue(LaunchConfiguration('restore_dock'), value_type=bool)}]),
        Node(package='minibot', executable='person_detector', output='screen',
             condition=IfCondition(LaunchConfiguration('vision')),
             parameters=[{'use_sim_time': simulated}]),
        Node(package='minibot', executable='speech', output='screen',
             condition=IfCondition(LaunchConfiguration('speech'))),
    ])
