"""Search and rescue on the real Unitree Go2 (or the same stack in Gazebo).

    # real robot (inside the Docker container); dry run first, then enable_motion:=true
    ros2 launch go2_real_bridge real_sar.launch.py mode:=nomap
    ros2 launch go2_real_bridge real_sar.launch.py mode:=known mission:=/missions/real_field enable_motion:=true

    # the same real-robot stack on the simulation (dev machine): fake_go2 emulates the
    # Go2 topics on top of Gazebo / CHAMP, mission_monitor referees
    ros2 launch go2_real_bridge real_sar.launch.py mode:=nomap sim_test:=true enable_motion:=true gui:=false

mode:=slam    go2_sar_slam   sar_slam   (A4: builds its own map, no victim positions)
mode:=prior   go2_sar_prior  sar_prior  (A3: prior map + estimated victim positions)
mode:=reflex  go2_sar_reflex sar_reflex (A2: no map, no localisation, no planner)
mode:=scout   go2_sar_scout  sar_scout  (A1: no map, scan-matching odometry)
             nomap / known are still accepted as aliases for slam / prior
camera:=host       the D435i driver runs outside (host realsense-ros, the default)
camera:=container  start realsense2_camera here (needs the USB device in the container)
camera:=none       no camera node (simulation, or started elsewhere)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)
    mode, sim = arg('mode'), arg('sim_test') == 'true'
    mode = {'nomap': 'slam', 'known': 'prior'}.get(mode, mode)   # old names still accepted
    if mode not in ('slam', 'prior', 'reflex', 'scout'):
        raise RuntimeError(f'mode must be slam, prior, reflex or scout, not {mode}')
    cfg = os.path.join(get_package_share_directory('go2_real_bridge'), 'config')
    common = os.path.join(cfg, 'real_common.yaml')
    mission_yaml = os.path.join(cfg, {'slam': 'real_nomap.yaml', 'prior': 'real_known.yaml',
                                     'reflex': 'real_reflex.yaml', 'scout': 'real_scout.yaml'}[mode])
    # the missions' own tuned parameters first, then the real-robot and simulation overrides
    if mode == 'scout':
        tuned = os.path.join(get_package_share_directory('go2_sar_scout'), 'config', 'scout_params.yaml')
    elif mode in ('slam', 'reflex'):
        tuned = os.path.join(get_package_share_directory('go2_sar_reflex' if mode == 'reflex' else 'go2_sar_slam'),
                             'config', 'reflex_params.yaml' if mode == 'reflex' else 'slam_params.yaml')
    else:
        tuned = os.path.join(get_package_share_directory('go2_sar_prior'), 'config', 'prior_params.yaml')
    files = [tuned, common, mission_yaml] + ([os.path.join(cfg, 'sim_test.yaml')] if sim else [])
    if arg('extra_params'):     # on-site tuning without rebuilding the image (loaded last)
        files.append(arg('extra_params'))
    base = {'use_sim_time': sim}
    out = []
    if sim:
        models = get_package_share_directory('disaster_models')
        world = arg('world') or os.path.join(models, 'worlds', 'sar_arena_5x8.sdf')
        out.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('unitree_go2_sim'), 'launch', 'unitree_go2_launch.py')),
            launch_arguments={'rviz': 'false', 'world': world, 'gui': arg('gui'),
                              'world_init_x': '0.0', 'world_init_y': '0.0', 'world_init_heading': '0.0'}.items()))
        out.append(Node(package='go2_real_bridge', executable='fake_go2', name='fake_go2', output='screen',
                        parameters=files + [base]))
    out.append(Node(package='go2_real_bridge', executable='go2_bridge', name='go2_bridge', output='screen',
                    parameters=files + [base, {'enable_motion': arg('enable_motion') == 'true'}]))
    if arg('camera') == 'container' and not sim:
        out.append(Node(package='realsense2_camera', executable='realsense2_camera_node', name='camera',
                        namespace='camera', output='screen',
                        parameters=[{'enable_color': True, 'enable_depth': False, 'enable_gyro': False,
                                     'enable_accel': False, 'rgb_camera.color_profile': '640,480,15'}]))
    out.append(Node(package='go2_sar_core', executable='person_detector', name='person_detector', output='screen',
                    # scout, like nomap, has no publisher for /sar/detector_enable --
                    # it predates that topic -- so YOLO has to start enabled or it
                    # never runs at all. Measured: with start_enabled False the
                    # explorer drove 5.3 m x 2.8 m and never once entered APPROACH,
                    # because not a single detection was ever published.
                    parameters=files + [base, {'start_enabled': mode in ('slam', 'scout')}]))
    trace = {'trace_file': arg('trace_file')} if arg('trace_file') else {}
    debug = {'debug': arg('debug') == 'true'}
    if mode == 'scout':
        # A1, the pre-bridge reactive explorer. It reports progress on
        # /sar/status rather than /sar/events, so an adapter translates for
        # the referee; the node itself is untouched.
        out.append(Node(package='go2_sar_scout', executable='sar_scout',
                        name='sar_scout', output='screen',
                        parameters=files + [base, debug]))
        out.append(Node(package='go2_sar_scout', executable='status_referee_adapter',
                        name='status_referee_adapter', output='screen', parameters=[base]))
    elif mode == 'reflex':
        # No map, no SLAM, no planner -- the fallback for a robot that cannot
        # run mapping. Same victim contract as the other two: it publishes
        # MISSION_START / SIT on /sar/events and really sits down.
        out.append(Node(package='go2_sar_reflex', executable='sar_reflex', name='sar_reflex', output='screen',
                        parameters=files + [base, debug, trace,
                                            {'expected_victims': int(arg('expected_victims'))}]))
    elif mode == 'slam':
        out.append(Node(package='go2_sar_slam', executable='sar_slam', name='sar_slam', output='screen',
                        parameters=files + [base, debug, trace,
                                            {'expected_victims': int(arg('expected_victims')),
                                             'debug_map_dir': arg('debug_map_dir')}]))
    else:
        mission = arg('mission')
        if not mission:
            mission = os.path.join(get_package_share_directory('disaster_models'), 'missions', 'arena_5x8') \
                if sim else '/missions/real_field'
        out.append(Node(package='go2_sar_prior', executable='sar_prior', name='sar_prior', output='screen',
                        parameters=files + [base, debug, trace,
                                            {'map_yaml': os.path.join(mission, 'map.yaml'),
                                             'victims_yaml': os.path.join(mission, 'victims_estimated.yaml')}]))
    if sim:     # referee (Gazebo ground truth) - simulation only
        mission = arg('mission') or os.path.join(get_package_share_directory('disaster_models'), 'missions',
                                                 'arena_5x8')
        # extra_params is appended so the referee can be retimed like every other
        # node. Without it the two modes were refereed over different durations
        # (nomap 300 s, known 180 s) with no way to equalise them from outside,
        # which makes the two scores incomparable -- and comparing them is the
        # whole point. The defaults below are unchanged; only the override is new.
        monitor_params = [base, {'ground_truth_topic': '/odom/ground_truth', 'success_distance': 1.0,
                                 'time_limit': 180.0 if mode == 'known' else 300.0,
                                 'ground_truth_yaml': os.path.join(mission, 'victims_ground_truth.yaml')}]
        if arg('extra_params'):
            monitor_params.append(arg('extra_params'))
        out.append(Node(package='go2_sar_referee', executable='mission_monitor', name='mission_monitor', output='screen',
                        parameters=monitor_params))
    return out


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='nomap', description='slam | prior | reflex | scout (nomap/known still accepted)'),
        DeclareLaunchArgument('mission', default_value='',
                              description='known: folder with map.yaml + victims_estimated.yaml '
                                          '(default real: /missions/real_field, sim: disaster_models arena_5x8)'),
        DeclareLaunchArgument('enable_motion', default_value='false',
                              description='false: dry run, no motion request reaches the robot'),
        DeclareLaunchArgument('expected_victims', default_value='4'),
        DeclareLaunchArgument('camera', default_value='host', description='host | container | none'),
        DeclareLaunchArgument('sim_test', default_value='false',
                              description='true: run on Gazebo through fake_go2 (dev machine)'),
        DeclareLaunchArgument('world', default_value='', description='sim_test: Gazebo world (default training arena)'),
        DeclareLaunchArgument('gui', default_value='true', description='sim_test: false = Gazebo server only'),
        DeclareLaunchArgument('extra_params', default_value='',
                              description='extra params file loaded last, e.g. /logs/my_params.yaml'),
        DeclareLaunchArgument('debug', default_value='false'),
        DeclareLaunchArgument('trace_file', default_value='', description='one CSV row per control tick'),
        DeclareLaunchArgument('debug_map_dir', default_value='', description='nomap: map snapshots'),
        OpaqueFunction(function=nodes),
    ])
