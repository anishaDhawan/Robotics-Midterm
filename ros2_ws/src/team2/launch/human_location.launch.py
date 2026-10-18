"""Start human detection under team2 using the running camera and camera-to-map TF."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Let the user choose where the YOLO model files are stored.
        DeclareLaunchArgument('model_dir', default_value=str(Path.home() / 'human_model')),
        # Use Gazebo's clock when running in simulation.
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        # Start the vision node and connect it to team2's TF and clock topics.
        Node(package='team2', executable='human_location_node', namespace='team2',
             parameters=[{'model_dir': LaunchConfiguration('model_dir'),
                          'use_sim_time': LaunchConfiguration('use_sim_time')}],
             remappings=[('/tf', '/team2/tf'), ('/tf_static', '/team2/tf_static'),
                         ('/clock', '/team2/clock')], output='screen'),
    ])
