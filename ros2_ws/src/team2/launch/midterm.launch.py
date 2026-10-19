import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directory
    team2_dir = get_package_share_directory('team2')

    # 1. Include your teammates' existing Nav2/SLAM launch file
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(team2_dir, 'launch', 'navigation_launch.py')
        )
    )

    # 2. Start the Master Controller (Grading Logic)
    master_controller_node = Node(
        package='team2',
        executable='navigation',
        name='navigation_node',
        output='screen'
    )

    # 3. Start the Frontier Explorer (Autonomous Mapping)
    explorer_node = Node(
        package='team2',
        executable='explore',
        name='explore_node',
        output='screen'
    )

    return LaunchDescription([
        nav2_launch,
        master_controller_node,
        explorer_node
    ])
