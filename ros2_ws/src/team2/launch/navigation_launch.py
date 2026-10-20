import os
import tempfile
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def _patch_params(params):
    # WIDE TURN FIX:
    # High scaling combined with a 0.35m inflation creates a "valley" of low cost 
    # perfectly in the center of hallways, forcing the robot away from corners.
    ROBOT_RADIUS = 0.17 
    INFLATION = 0.35
    SCALING = 4.0 
    
    g = params['global_costmap']['global_costmap']['ros__parameters']
    g.update(
        rolling_window=False, width=40, height=40, origin_x=-20.0, origin_y=-20.0,
        resolution=0.05, track_unknown_space=True, robot_radius=ROBOT_RADIUS,
        plugins=['obstacle_layer', 'inflation_layer'],
    )
    g.pop('static_layer', None)
    g.setdefault('inflation_layer', {}).update(inflation_radius=INFLATION, cost_scaling_factor=SCALING)
    g.setdefault('obstacle_layer', {}).update({'observation_sources': 'scan'})
    g['obstacle_layer'].setdefault('scan', {}).update({'topic': '/scan', 'data_type': 'LaserScan', 'clearing': True, 'marking': True})

    l = params['local_costmap']['local_costmap']['ros__parameters']
    l['robot_radius'] = ROBOT_RADIUS
    l.setdefault('inflation_layer', {}).update(inflation_radius=INFLATION, cost_scaling_factor=SCALING)
    for layer in ('voxel_layer', 'obstacle_layer'):
        if layer in l:
            l[layer].update({'observation_sources': 'scan'})
            l[layer].setdefault('scan', {}).update({'topic': '/scan', 'data_type': 'LaserScan', 'clearing': True, 'marking': True})

    params['planner_server']['ros__parameters']['GridBased'].update(allow_unknown=True, use_astar=True)
    params['planner_server']['ros__parameters']['expected_planner_frequency'] = 2.0
    if 'controller_server' in params:
        params['controller_server']['ros__parameters']['controller_frequency'] = 5.0

    return params

def _launch_setup(context):
    nav2_dir = get_package_share_directory('nav2_bringup')
    with open(os.path.join(nav2_dir, 'params', 'nav2_params.yaml')) as f:
        params = _patch_params(yaml.safe_load(f))

    patched = os.path.join(tempfile.gettempdir(), 'team2_nav2_params.yaml')
    with open(patched, 'w') as f:
        yaml.safe_dump(params, f)

    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={'namespace': '', 'use_namespace': 'False', 'slam': 'True', 'map': '', 'use_sim_time': 'True', 'params_file': patched, 'autostart': 'True'}.items())]

def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_launch_setup)])
