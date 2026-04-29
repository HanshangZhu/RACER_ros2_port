"""Per-drone simulator stack (light weight, cmd2odom path).

ROS 1 equivalent: simulator_light.xml. The original swarm demo actually
includes the *light* variant (simulator.xml in ROS 1 has the full SO(3)
controller + disturbance generator; the swarm demo opts for cmd2odom + depth
sim to keep 4 drones cheap). That's what we mirror here.

Per-drone nodes:
  * poscmd_2_odom       - command-to-odom loopback
  * odom_visualization  - RViz mesh/trail publisher
  * pcl_render_node     - software depth / point-cloud simulator
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def _args():
    return [
        DeclareLaunchArgument('drone_id'),
        DeclareLaunchArgument('drone_num'),
        DeclareLaunchArgument('init_x'),
        DeclareLaunchArgument('init_y'),
        DeclareLaunchArgument('init_z'),
        DeclareLaunchArgument('map_size_x'),
        DeclareLaunchArgument('map_size_y'),
        DeclareLaunchArgument('map_size_z'),
        DeclareLaunchArgument('odometry_topic'),
        DeclareLaunchArgument('robot_type', default_value='quadrotor',
                              description='quadrotor | quadruped. Quadruped uses '
                                          'quadruped_simulator/cmd2base and pins z.'),
        DeclareLaunchArgument('stand_height', default_value='0.30'),
    ]


def _build(context):
    def P(name):
        return LaunchConfiguration(name).perform(context)

    drone_id = P('drone_id')
    drone_num = P('drone_num')
    init_x, init_y, init_z = float(P('init_x')), float(P('init_y')), float(P('init_z'))
    map_x, map_y, map_z = float(P('map_size_x')), float(P('map_size_y')), float(P('map_size_z'))
    odom_topic = P('odometry_topic')
    quadruped = P('robot_type').lower() == 'quadruped'
    stand_height = float(P('stand_height'))

    # For quadrupeds the sim's init_z is pinned to stand_height regardless of
    # what the caller passed (the caller's init_z is the drone-demo value).
    sim_init_z = stand_height if quadruped else init_z

    # Load camera intrinsics from the installed local_sensing/params/camera.yaml.
    cam_yaml = os.path.join(
        get_package_share_directory('local_sensing'), 'params', 'camera.yaml'
    )

    if quadruped:
        base_integrator = Node(
            package='quadruped_simulator',
            executable='cmd2base',
            name=f'cmd2base_{drone_id}',
            output='screen',
            parameters=[{
                'drone_id': int(drone_id),
                'init_x': init_x,
                'init_y': init_y,
                'stand_height': stand_height,
                'v_fwd_max': 1.0,
                'v_lat_max': 0.5,
                'omega_max': 1.0,
                'rate_hz': 50.0,
            }],
            remappings=[
                ('command', f'/planning/pos_cmd_{drone_id}'),
                ('odometry', odom_topic),
            ],
        )
    else:
        base_integrator = Node(
            package='poscmd_2_odom',
            executable='poscmd_2_odom',
            name=f'poscmd_2_odom_{drone_id}',
            output='screen',
            parameters=[{
                'drone_id': int(drone_id),
                'init_x': init_x,
                'init_y': init_y,
                'init_z': init_z,
            }],
            remappings=[
                ('~/command', f'/planning/pos_cmd_{drone_id}'),
                ('~/odometry', odom_topic),
            ],
        )

    odom_vis = Node(
        package='odom_visualization',
        executable='odom_visualization',
        name=f'odom_visualization_{drone_id}',
        output='screen',
        parameters=[{
            'drone_id': int(drone_id),
            'drone_num': int(drone_num),
            'color.a': 1.0,
            'color.r': 0.0,
            'color.g': 0.0,
            'color.b': 1.0,
            'covariance_scale': 100.0,
            'robot_scale': 1.0,
        }],
        remappings=[('~/odom', odom_topic)],
    )

    pcl_render = Node(
        package='local_sensing',
        executable='pcl_render_node',
        name=f'pcl_render_node_{drone_id}',
        output='screen',
        parameters=[
            cam_yaml,
            {
                'sensing_horizon': 5.0,
                'sensing_rate': 10.0,
                'estimation_rate': 10.0,
                'map.x_size': map_x,
                'map.y_size': map_y,
                'map.z_size': map_z,
                'map.init_x': init_x,
                'map.init_y': init_y,
                'map.init_z': sim_init_z,
                'map.drone_id': int(drone_id),
            },
        ],
        remappings=[
            ('~/global_map', '/map_generator/global_cloud'),
            ('~/odometry', odom_topic),
            # The per-drone topic fan-out: the node publishes to
            # /pcl_render_node/<topic>; remap to /pcl_render_node/<topic>_<id>.
            ('/pcl_render_node/depth', f'/pcl_render_node/depth_{drone_id}'),
            ('/pcl_render_node/sensor_pose', f'/pcl_render_node/sensor_pose_{drone_id}'),
            ('/pcl_render_node/odom', f'/pcl_render_node/odom_{drone_id}'),
        ],
    )

    return [base_integrator, odom_vis, pcl_render]


def generate_launch_description():
    return LaunchDescription(_args() + [OpaqueFunction(function=_build)])
