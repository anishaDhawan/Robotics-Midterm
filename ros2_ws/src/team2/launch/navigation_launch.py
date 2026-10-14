import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    params_file = os.path.join(nav2_bringup_dir, 'params', 'nav2_params.yaml')

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'namespace': '',             # <-- THE FIX: Set to empty string
            'use_namespace': 'False',
            'slam': 'True',
            'map': '',
            'use_sim_time': 'True',
            'params_file': params_file,
            'autostart': 'True',
        }.items()
    )

    return LaunchDescription([
        bringup_launch
    ])