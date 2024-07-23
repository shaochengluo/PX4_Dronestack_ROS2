from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Define launch arguments
    return LaunchDescription([
        DeclareLaunchArgument('fcu_url', default_value='/dev/ttyACM0:115200'),
        DeclareLaunchArgument('gcs_url', default_value=''),
        DeclareLaunchArgument('tgt_system', default_value='1'),
        DeclareLaunchArgument('tgt_component', default_value='1'),
        DeclareLaunchArgument('log_output', default_value='screen'),
        DeclareLaunchArgument('fcu_protocol', default_value='v2.0'),
        DeclareLaunchArgument('respawn_mavros', default_value='false'),
        DeclareLaunchArgument('namespace', default_value='mavros'),

        # Include the MAVROS launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('mavros'),
                    'launch',
                    'node.launch.py'
                )
            ),
            launch_arguments={
                'pluginlists_yaml': os.path.join(
                    get_package_share_directory('mavros'),
                    'launch',
                    'px4_pluginlists.yaml'
                ),
                'config_yaml': os.path.join(
                    get_package_share_directory('mavros'),
                    'launch',
                    'px4_config.yaml'
                ),
                'fcu_url': LaunchConfiguration('fcu_url'),
                'gcs_url': LaunchConfiguration('gcs_url'),
                'tgt_system': LaunchConfiguration('tgt_system'),
                'tgt_component': LaunchConfiguration('tgt_component'),
                'log_output': LaunchConfiguration('log_output'),
                'fcu_protocol': LaunchConfiguration('fcu_protocol'),
                'respawn_mavros': LaunchConfiguration('respawn_mavros'),
                'namespace': LaunchConfiguration('namespace')
            }.items()
        )
    ])

if __name__ == '__main__':
    generate_launch_description()

