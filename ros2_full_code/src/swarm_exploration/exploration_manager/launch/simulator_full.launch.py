"""Per-drone FULL (heavy) simulator stack.

ROS 1 equivalent: simulator.xml (the heavy variant that the swarm demo does
*not* include — the swarm uses simulator_light.xml). Port opted to mirror the
light variant first; this file is the optional heavy-sim path, selected via
`sim_type:=full` on swarm_exploration.launch.py.

Heavy pipeline per drone (relative topics, remapped into per-drone global names):

    so3_control  --so3_cmd-->  quadrotor_simulator_so3
                                     |     |
                                  (odom)  (imu)
                                     |     |
                                     v     v
                             so3_disturbance   so3_control
                                     |
                                noisy_odom ----> so3_control
                                     |
                              {odom_prefix}_<id> (exposed to planner)
                             force/moment_disturbance -> quadrotor_simulator_so3

Also starts the per-drone odom_visualization and pcl_render_node (identical to
simulator.launch.py so downstream RViz sees the same visualisation topics).
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
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
    ]


def _build(context):
    def P(name):
        return LaunchConfiguration(name).perform(context)

    drone_id = P('drone_id')
    drone_num = P('drone_num')
    init_x, init_y, init_z = float(P('init_x')), float(P('init_y')), float(P('init_z'))
    map_x, map_y, map_z = float(P('map_size_x')), float(P('map_size_y')), float(P('map_size_z'))
    odom_topic = P('odometry_topic')  # exposed to planner; produced by disturbance generator

    # Per-drone internal bus (keeps multi-drone instances from colliding).
    bus = f'/drone_{drone_id}'
    sim_odom = f'{bus}/sim_odom'
    sim_imu = f'{bus}/sim_imu'
    so3_cmd = f'{bus}/so3_cmd'
    f_dist = f'{bus}/force_disturbance'
    m_dist = f'{bus}/moment_disturbance'
    correction = f'{bus}/correction'
    position_cmd = f'/planning/pos_cmd_{drone_id}'

    cam_yaml = os.path.join(
        get_package_share_directory('local_sensing'), 'params', 'camera.yaml'
    )
    so3_share = get_package_share_directory('so3_control')
    gains_yaml = os.path.join(so3_share, 'config', 'gains_hummingbird.yaml')
    corrections_yaml = os.path.join(so3_share, 'config', 'corrections_hummingbird.yaml')

    # Physics -----------------------------------------------------------------
    quadrotor = Node(
        package='so3_quadrotor_simulator',
        executable='quadrotor_simulator_so3',
        name=f'quadrotor_simulator_so3_{drone_id}',
        output='screen',
        parameters=[{
            'rate.odom': 200.0,
            'simulator.init_state_x': init_x,
            'simulator.init_state_y': init_y,
            'simulator.init_state_z': init_z,
            'quadrotor_name': f'drone_{drone_id}',
        }],
        remappings=[
            ('odom', sim_odom),
            ('imu', sim_imu),
            ('cmd', so3_cmd),
            ('force_disturbance', f_dist),
            ('moment_disturbance', m_dist),
        ],
    )

    # Disturbance / noisy odometry -------------------------------------------
    disturbance = Node(
        package='so3_disturbance_generator',
        executable='so3_disturbance_generator',
        name=f'so3_disturbance_generator_{drone_id}',
        output='screen',
        remappings=[
            ('odom', sim_odom),
            ('noisy_odom', odom_topic),
            ('correction', correction),
            ('force_disturbance', f_dist),
            ('moment_disturbance', m_dist),
        ],
    )

    # SO(3) controller (rclcpp component loaded into per-drone container) ----
    so3_control = ComposableNodeContainer(
        name=f'so3_control_container_{drone_id}',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package='so3_control',
                plugin='so3_control::SO3ControlComponent',
                name=f'so3_control_{drone_id}',
                parameters=[
                    gains_yaml,
                    corrections_yaml,
                    {
                        'mass': 0.98,
                        'use_angle_corrections': False,
                        'use_external_yaw': False,
                        'gains.rot.z': 1.0,
                        'gains.ang.z': 0.1,
                        'quadrotor_name': f'drone_{drone_id}',
                    },
                ],
                remappings=[
                    ('odom', odom_topic),
                    ('position_cmd', position_cmd),
                    ('imu', sim_imu),
                    ('so3_cmd', so3_cmd),
                    ('motors', f'{bus}/motors'),
                    ('corrections', f'{bus}/corrections'),
                ],
            ),
        ],
    )

    # Visualisation + depth rendering (same as light sim) --------------------
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
                'map.drone_id': int(drone_id),
            },
        ],
        remappings=[
            ('~/global_map', '/map_generator/global_cloud'),
            ('~/odometry', odom_topic),
            ('/pcl_render_node/depth', f'/pcl_render_node/depth_{drone_id}'),
            ('/pcl_render_node/sensor_pose', f'/pcl_render_node/sensor_pose_{drone_id}'),
            ('/pcl_render_node/odom', f'/pcl_render_node/odom_{drone_id}'),
        ],
    )

    return [quadrotor, disturbance, so3_control, odom_vis, pcl_render]


def generate_launch_description():
    return LaunchDescription(_args() + [OpaqueFunction(function=_build)])
