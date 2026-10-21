import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    team2_dir = get_package_share_directory('team2')
    nav_launch_path = os.path.join(team2_dir, 'launch', 'navigation_launch.py')

    return LaunchDescription([
        # Force global simulation clock usage for ALL nodes
        SetParameter(name='use_sim_time', value=True),

        SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity} {time}] [{name}]: {message}'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav_launch_path),
            launch_arguments={'use_sim_time': 'true'}.items()
        ),
        Node(
            package='team2',
            executable='navigation',
            name='navigation_node',
            output='screen'
        ),
        Node(
            package='team2',
            executable='human_location',
            name='human_location_node',
            output='screen'
        ),
        Node(
            package='team2',
            executable='explore',
            name='explore_node',
            output='screen'
        ),
    ])