"""Start human detection using the selected Python environment and existing camera."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model_dir', default_value=str(Path.home() / 'human_model')),
        # ONNX Runtime is installed in this environment, not necessarily in ROS's Python.
        DeclareLaunchArgument('vision_python', default_value=str(
            Path.home() / 'venvs/team2_vision/bin/python')),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        # Use the Python environment that has the YOLO runtime installed.
        ExecuteProcess(cmd=[
            LaunchConfiguration('vision_python'), '-m', 'team2.human_location_node',
            '--ros-args', '-r', '__ns:=/team2',
            '-r', '/tf:=/team2/tf', '-r', '/tf_static:=/team2/tf_static',
            '-r', '/clock:=/team2/clock',
            '-p', ['model_dir:=', LaunchConfiguration('model_dir')],
            '-p', ['use_sim_time:=', LaunchConfiguration('use_sim_time')],
        ], output='screen'),
    ])
