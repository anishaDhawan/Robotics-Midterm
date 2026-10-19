"""Start the team mission using an already-running RGB-D robot and world."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('team2'))
    # Every node must use the same simulation clock and transform topics.
    remaps = [('/clock', '/team2/clock'), ('/tf', '/team2/tf'),
              ('/tf_static', '/team2/tf_static')]
    return LaunchDescription([
        # Match the world pose where wheel odometry originally started.
        DeclareLaunchArgument('odom_origin_x', default_value='-4.5'),
        DeclareLaunchArgument('odom_origin_y', default_value='3.5'),
        DeclareLaunchArgument('odom_origin_yaw', default_value='0.0'),
        DeclareLaunchArgument('model_dir', default_value=str(Path.home() / 'human_model')),
        DeclareLaunchArgument('vision_python', default_value=str(
            Path.home() / 'venvs/team2_vision/bin/python')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(share / 'launch/navigation_launch.py')),
            launch_arguments={name: LaunchConfiguration(name) for name in
                              ('odom_origin_x', 'odom_origin_y', 'odom_origin_yaw')}.items()),
        # Navigation handles goals, status updates and the final movement commands.
        Node(package='team2', executable='navigation', namespace='team2',
             parameters=[{'use_sim_time': True}], remappings=remaps, output='screen'),
        # Exploration suggests destinations; navigation decides when to use them.
        Node(package='team2', executable='explore', namespace='team2',
             parameters=[{'use_sim_time': True}], remappings=remaps, output='screen'),
        # Turn depth images into obstacle points for Nav2.
        Node(package='team2', executable='depth_obstacles', namespace='team2',
             parameters=[{'use_sim_time': True}], remappings=remaps, output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(share / 'launch/human_location.launch.py')),
            launch_arguments={
                'model_dir': LaunchConfiguration('model_dir'),
                'vision_python': LaunchConfiguration('vision_python'),
                'use_sim_time': 'true',
            }.items()),
    ])
