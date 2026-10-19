import os
import math
import tempfile
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch.launch_description_sources import PythonLaunchDescriptionSource

def _patch_params(params):
    # Leave a safety margin around obstacles when planning the route.
    ROBOT_RADIUS = 0.17 
    INFLATION = 0.35
    SCALING = 4.0 
    
    # Start with the SLAM map, then add current sensor obstacles and safety margins.
    g = params['global_costmap']['global_costmap']['ros__parameters']
    g.update(
        # Keep room to plan into unexplored areas instead of shrinking to the SLAM map.
        rolling_window=True, width=40, height=40, origin_x=-20.0, origin_y=-20.0,
        resolution=0.05, track_unknown_space=True, robot_radius=ROBOT_RADIUS,
        plugins=['static_layer', 'obstacle_layer', 'inflation_layer'],
    )
    # Keep mapped walls even when they are outside the current camera/LiDAR view.

    g['static_layer'] = {
        'plugin': 'nav2_costmap_2d::StaticLayer',
        'enabled': True,
        'map_topic': '/team2/map',
        'map_subscribe_transient_local': True,
        'subscribe_to_updates': True,
        'use_maximum': False,
    }
    g.setdefault('inflation_layer', {}).update(inflation_radius=INFLATION, cost_scaling_factor=SCALING)
    g.setdefault('obstacle_layer', {}).update({'observation_sources': 'scan'})
    g['obstacle_layer'].setdefault('scan', {}).update({'topic': '/team2/scan', 'data_type': 'LaserScan', 'clearing': True, 'marking': True})

    # The local costmap checks nearby obstacles while the robot follows its path.
    l = params['local_costmap']['local_costmap']['ros__parameters']
    l['robot_radius'] = ROBOT_RADIUS
    l.setdefault('inflation_layer', {}).update(inflation_radius=INFLATION, cost_scaling_factor=SCALING)
    for layer in ('voxel_layer', 'obstacle_layer'):
        if layer in l:
            l[layer].update({'observation_sources': 'scan'})
            l[layer].setdefault('scan', {}).update({'topic': '/team2/scan', 'data_type': 'LaserScan', 'clearing': True, 'marking': True})

    params['planner_server']['ros__parameters']['GridBased'].update(allow_unknown=True, use_astar=True)
    params['planner_server']['ros__parameters']['expected_planner_frequency'] = 2.0
    if 'controller_server' in params:
        params['controller_server']['ros__parameters']['controller_frequency'] = 5.0
        # Follow a point ahead on the path instead of scoring near-stationary turns.
        # Nav2 Humble's Regulated Pure Pursuit still checks the costmap for collisions.
        params['controller_server']['ros__parameters']['FollowPath'] = {
            'plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
            'desired_linear_vel': 0.26,
            'lookahead_dist': 0.4,
            'use_velocity_scaled_lookahead_dist': False,
            'use_interpolation': True,
            'use_collision_detection': True,
            'max_allowed_time_to_collision_up_to_carrot': 1.5,
            'use_regulated_linear_velocity_scaling': True,
            'regulated_linear_scaling_min_radius': 0.6,
            'regulated_linear_scaling_min_speed': 0.05,
            'use_cost_regulated_linear_velocity_scaling': True,
            'cost_scaling_dist': 0.3,
            'cost_scaling_gain': 1.0,
            'inflation_cost_scaling_factor': SCALING,
            'use_rotate_to_heading': True,
            'rotate_to_heading_min_angle': 0.5,
            'rotate_to_heading_angular_vel': 0.4,
            'max_angular_accel': 1.0,
            'min_approach_linear_velocity': 0.03,
            'approach_velocity_scaling_dist': 0.5,
            'transform_tolerance': 0.2,
        }
        # Assignment goals specify a position, not a final facing direction.
        # Avoid turning away from the camera view just to face east on arrival.
        params['controller_server']['ros__parameters']['general_goal_checker'].update(
            yaw_goal_tolerance=math.pi)
    params['velocity_smoother']['ros__parameters'].update(
        max_velocity=[0.26, 0.0, 0.5], min_velocity=[-0.18, 0.0, -0.5])
    params['behavior_server']['ros__parameters'].update(
        max_rotational_vel=0.5, min_rotational_vel=0.1)

    # A zero expected_update_rate disables depth-arrival timeouts in Nav2.
    # Keep depth in a separate layer: LiDAR rays above a small floor object
    # must not erase the camera's obstacle mark.
    for costmap in (g, l):
        plugins = costmap['plugins']
        if 'depth_layer' not in plugins:
            plugins.insert(plugins.index('inflation_layer'), 'depth_layer')
        costmap['depth_layer'] = {
            'plugin': 'nav2_costmap_2d::ObstacleLayer',
            'enabled': True,
            'combination_method': 1,
            'observation_sources': 'depth_marks depth_clear',
            'depth_marks': {
                'topic': '/team2/camera/obstacles', 'data_type': 'PointCloud2',
                'marking': True, 'clearing': False,
                'min_obstacle_height': -1.0, 'max_obstacle_height': 5.0,
                'obstacle_min_range': 0.0, 'obstacle_max_range': 3.5,
                'expected_update_rate': 0.0, 'observation_persistence': 0.0,
            },
            'depth_clear': {
                'topic': '/team2/camera/clearing_points', 'data_type': 'PointCloud2',
                'marking': False, 'clearing': True,
                # Keep floor endpoints for clearing; the Python node already
                # removes the floor from the separate marking cloud.
                'min_obstacle_height': -1.0, 'max_obstacle_height': 5.0,
                'raytrace_min_range': 0.0, 'raytrace_max_range': 3.5,
                'expected_update_rate': 0.0, 'observation_persistence': 0.0,
            },
        }
    params['bt_navigator']['ros__parameters']['odom_topic'] = '/team2/odom'
    return params

def _launch_setup(context):
    nav2_dir = get_package_share_directory('nav2_bringup')
    # These are the world coordinates of wheel odometry's (0, 0), not the human.
    origin = {axis: float(LaunchConfiguration('odom_origin_' + axis).perform(context))
              for axis in ('x', 'y', 'yaw')}
    if not all(math.isfinite(value) for value in origin.values()):
        raise ValueError('Odometry origin values must be finite numbers.')
    with open(os.path.join(nav2_dir, 'params', 'nav2_params.yaml')) as f:
        params = _patch_params(yaml.safe_load(f))

    # Keep the installed SLAM defaults, but give SLAM world-aligned odometry.
    # The first scan then starts at the robot's real starting coordinates.
    slam_dir = get_package_share_directory('slam_toolbox')
    with open(os.path.join(slam_dir, 'config', 'mapper_params_online_sync.yaml')) as f:
        slam_params = yaml.safe_load(f)['slam_toolbox']
    slam_params['ros__parameters'].update(
        odom_frame='aligned_odom', map_frame='map',
        base_frame='base_footprint', scan_topic='/team2/scan',
        # Use closer-spaced scans so turns do not leave large gaps in the map.
        minimum_travel_distance=0.1, minimum_travel_heading=0.1,
        minimum_time_interval=0.2, max_laser_range=3.5)
    # Nav2 Humble checks for this short key before forwarding SLAM parameters.
    params['slam_toolbox'] = {'ros__parameters': {'use_sim_time': True}}
    # SLAM reads this file directly, so also place its settings under team2.
    # Use a namespace section: Nav2 wraps this file again for its own nodes.
    params['team2'] = {'slam_toolbox': slam_params}

    # Give Nav2 these settings without overwriting its installed configuration.
    with tempfile.NamedTemporaryFile(mode='w', prefix='team2_nav2_', suffix='.yaml', delete=False) as f:
        yaml.safe_dump(params, f)
        patched = f.name

    # All internal topics use team2. Professor goal/status topics stay global.
    return [GroupAction([
        SetRemap(src='/clock', dst='/team2/clock'),
        SetRemap(src='/scan', dst='/team2/scan'),
        SetRemap(src='/odom', dst='/team2/odom'),
        SetRemap(src='/map', dst='/team2/map'),
        SetRemap(src='/tf', dst='/team2/tf'),
        SetRemap(src='/tf_static', dst='/team2/tf_static'),
        # Wheel odometry starts at zero wherever Gazebo spawns the robot.
        # This fixed link supplies that starting position and facing direction.
        # SLAM owns map -> aligned_odom; the simulator still owns odom -> base.
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='odom_origin', namespace='team2', output='screen',
             arguments=['--x', str(origin['x']), '--y', str(origin['y']),
                        '--z', '0', '--yaw', str(origin['yaw']),
                        '--pitch', '0', '--roll', '0',
                        '--frame-id', 'aligned_odom', '--child-frame-id', 'odom'],
             parameters=[{'use_sim_time': True}]),
        # Scope these rules to the two final Nav2 velocity publishers.
        # Leave the controller-to-smoother connection unchanged.
        SetRemap(src='velocity_smoother:cmd_vel_smoothed', dst='/team2/nav2_cmd_vel'),
        SetRemap(src='behavior_server:cmd_vel', dst='/team2/nav2_cmd_vel'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_dir, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'namespace': 'team2', 'use_namespace': 'True', 'slam': 'True',
                'map': '', 'use_sim_time': 'True', 'params_file': patched,
                'autostart': 'True', 'use_composition': 'False',
            }.items()),
    ])]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('odom_origin_x', default_value='-4.5'),
        DeclareLaunchArgument('odom_origin_y', default_value='3.5'),
        DeclareLaunchArgument('odom_origin_yaw', default_value='0.0'),
        OpaqueFunction(function=_launch_setup),
    ])
