"""Start a House world with a Waffle robot that has colour and depth cameras.
Original model/URDF: turtlebot3_gazebo (ROBOTIS), Apache-2.0.
Navigation and mapping must be started separately.
"""
from pathlib import Path
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Find the installed camera settings and TurtleBot3 simulation files.
    config = Path(get_package_share_directory('team2')) / 'config'
    share = Path(get_package_share_directory('turtlebot3_gazebo'))
    gz = Path(get_package_share_directory('ros_gz_sim'))
    # Keep existing paths so installed House furniture can still be found.
    model_paths = [
        *os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '').split(os.pathsep),
        str(share / 'models'),
        str(Path.home() / '.gazebo/models'),
        str(Path.home() / 'gazebo_models'),
        str(Path.home() / 'ign_models'),
    ]
    # Remove empty and repeated entries before passing the paths to Gazebo.
    model_paths = os.pathsep.join(dict.fromkeys(path for path in model_paths if path))
    return LaunchDescription([
        # Tell Gazebo where to find the robot and House models.
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', model_paths),
        # Start the House simulation server.
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gz / 'launch/gz_sim.launch.py')),
            launch_arguments={'gz_args': '-r -s -v2 ' + str(share / 'worlds/turtlebot3_house.world')}.items()),
        # Publish how the robot's body parts and camera are connected.
        Node(package='robot_state_publisher', executable='robot_state_publisher', namespace='team2',
            parameters=[{'robot_description': (config / 'waffle_rgbd.urdf').read_text(),
                         'use_sim_time': True, 'frame_prefix': ''}],
            remappings=[('/tf', '/team2/tf'), ('/tf_static', '/team2/tf_static'), ('/clock','/team2/clock')]),
        # Place the Waffle robot in the House world.
        Node(package='ros_gz_sim', executable='create', namespace='team2',
            arguments=['-world','default','-name','waffle','-file',str(config / 'waffle_rgbd.sdf'),
                       '-x','-4.5','-y','3.5','-z','0.01']),
        # Pass sensor data and movement commands between Gazebo and ROS.
        Node(package='ros_gz_bridge', executable='parameter_bridge', namespace='team2',
            parameters=[{'config_file': str(config / 'rgbd_bridge.yaml')}]),
    ])
