"""Headless stand-in sim: ign gazebo server + ros_gz bridge + Duburi sim nodes.

ros2 launch duburi_sim sim.launch.py arena:=default camera:=false rtf:=1.0 \
    detection_source:=ground_truth battery_start:=100.0 stack:=full
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from duburi_sim.arena import load_arena
from duburi_sim.world_gen import THRUSTERS, generate_world


def _setup(context):
    arena_name = LaunchConfiguration('arena').perform(context)
    camera = LaunchConfiguration('camera').perform(context).lower() == 'true'
    rtf = float(LaunchConfiguration('rtf').perform(context))
    stack = LaunchConfiguration('stack').perform(context)
    det_src = LaunchConfiguration('detection_source').perform(context)
    battery_start = float(LaunchConfiguration('battery_start').perform(context))
    verbose = LaunchConfiguration('gz_verbose').perform(context)

    arena = load_arena(arena_name)
    out_dir = os.path.join(os.environ.get('ROS_HOME', os.path.expanduser('~/.ros')), 'duburi_sim')
    os.makedirs(out_dir, exist_ok=True)
    world_file = os.path.join(out_dir, f'{os.path.basename(arena_name).replace(".yaml", "")}'
                                       f'{"_cam" if camera else ""}.sdf')
    with open(world_file, 'w') as f:
        f.write(generate_world(arena, camera=camera, rtf=rtf))

    gz_cmd = ['ign', 'gazebo', '-s', '-r', '-v', verbose]
    if camera:
        gz_cmd.append('--headless-rendering')
    gz_cmd.append(world_file)
    gz = ExecuteProcess(cmd=gz_cmd, output='screen', name='gz_server')

    bridge_args = [
        '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        '/model/duburi/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
        '/duburi/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
    ]
    remaps = [('/model/duburi/odometry', '/duburi/odom')]
    for n in THRUSTERS:
        gz_topic = f'/model/duburi/joint/{n}_joint/cmd_thrust'
        bridge_args.append(f'{gz_topic}@std_msgs/msg/Float64]ignition.msgs.Double')
        remaps.append((gz_topic, f'/duburi/thrusters/{n}'))
    if camera:
        bridge_args.append('/duburi/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image')
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', name='duburi_bridge',
                  arguments=bridge_args, remappings=remaps, output='screen',
                  parameters=[{'use_sim_time': True}])

    from ament_index_python.packages import get_package_share_directory
    params = os.path.join(get_package_share_directory('duburi_sim'), 'config', 'sim.yaml')
    arena_file = arena_name if os.path.isfile(arena_name) else \
        os.path.join(get_package_share_directory('duburi_sim'), 'config', 'arenas', f'{arena_name}.yaml')
    common = [params, {'use_sim_time': True}]

    def node(exe, name, extra=None, remap=None):
        return Node(package='duburi_sim', executable=exe, name=name, output='screen',
                    parameters=common + ([extra] if extra else []), remappings=remap or [])

    actions = [gz, bridge,
               node('controller_node', 'duburi_controller',
                    remap=[('odom', '/duburi/odom'), ('setpoint', '/duburi/controller/setpoint'),
                           ('mode', '/duburi/controller/mode')])]
    if stack == 'full':
        actions += [
            node('vehicle_server', 'duburi_vehicle_server', {'arena_file': arena_file}),
            node('battery_sim', 'duburi_battery', {'initial_percent': battery_start}),
            node('detector_node', 'duburi_detector',
                 {'arena_file': arena_file, 'detection_source': det_src, 'camera_enabled': camera}),
            node('referee_node', 'duburi_referee', {'arena_file': arena_file}),
        ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('arena', default_value='default', description='arena name or yaml path'),
        DeclareLaunchArgument('camera', default_value='false', description='render the front camera'),
        DeclareLaunchArgument('rtf', default_value='1.0', description='real-time factor (0 = max)'),
        DeclareLaunchArgument('stack', default_value='full', description='minimal | full'),
        DeclareLaunchArgument('detection_source', default_value='ground_truth',
                              description='ground_truth | camera'),
        DeclareLaunchArgument('battery_start', default_value='100.0'),
        DeclareLaunchArgument('gz_verbose', default_value='1'),
        OpaqueFunction(function=_setup),
    ])
