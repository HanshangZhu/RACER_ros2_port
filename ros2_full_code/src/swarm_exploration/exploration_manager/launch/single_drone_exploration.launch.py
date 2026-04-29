"""Per-drone stack: planner + traj_server (+ simulator when simulation:=true).

ROS 1 equivalent: single_drone_exploration.xml. Used once per drone by
swarm_exploration.launch.py.
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import os


def _args():
    return [
        DeclareLaunchArgument('drone_id', default_value='1'),
        DeclareLaunchArgument('drone_num', default_value='1'),
        DeclareLaunchArgument('init_x', default_value='0.0'),
        DeclareLaunchArgument('init_y', default_value='0.0'),
        DeclareLaunchArgument('init_z', default_value='1.0'),
        DeclareLaunchArgument('map_size_x', default_value='35.0'),
        DeclareLaunchArgument('map_size_y', default_value='35.0'),
        DeclareLaunchArgument('map_size_z', default_value='3.5'),
        DeclareLaunchArgument('odom_prefix', default_value='/state_ukf/odom'),
        DeclareLaunchArgument('simulation', default_value='true'),
        DeclareLaunchArgument('sim_type', default_value='light',
                              description='light (cmd2odom) | full (SO3 physics + controller).'),
        DeclareLaunchArgument('cx', default_value='324.0879821777344'),
        DeclareLaunchArgument('cy', default_value='239.10362243652344'),
        DeclareLaunchArgument('fx', default_value='385.69793701171875'),
        DeclareLaunchArgument('fy', default_value='385.69793701171875'),
        DeclareLaunchArgument('max_vel', default_value='1.5'),
        DeclareLaunchArgument('max_acc', default_value='1.0'),
        DeclareLaunchArgument('robot_type', default_value='quadrotor',
                              description='quadrotor | quadruped. Quadruped swaps the '
                                          'per-drone sim for quadruped_simulator/cmd2base '
                                          'and applies ground-plane tuning to the planner.'),
        DeclareLaunchArgument('stand_height', default_value='0.30',
                              description='quadruped: base-link height (m) above ground.'),
    ]


def _build(context):
    def P(name):
        return LaunchConfiguration(name).perform(context)

    drone_id = P('drone_id')
    drone_num = P('drone_num')
    odom_prefix = P('odom_prefix')
    odometry_topic = f'{odom_prefix}_{drone_id}'
    sensor_pose_topic = f'/pcl_render_node/sensor_pose_{drone_id}'
    depth_topic = f'/pcl_render_node/depth_{drone_id}'
    cloud_topic = f'/pcl_render_node/cloud_{drone_id}'

    share = get_package_share_directory('exploration_manager')

    planner_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'single_drone_planner.launch.py')),
        launch_arguments={
            'drone_id': drone_id,
            'drone_num': drone_num,
            'map_size_x': P('map_size_x'),
            'map_size_y': P('map_size_y'),
            'map_size_z': P('map_size_z'),
            'odometry_topic': odometry_topic,
            'sensor_pose_topic': sensor_pose_topic,
            'depth_topic': depth_topic,
            'cloud_topic': cloud_topic,
            'cx': P('cx'),
            'cy': P('cy'),
            'fx': P('fx'),
            'fy': P('fy'),
            'max_vel': P('max_vel'),
            'max_acc': P('max_acc'),
            'simulation': P('simulation'),
            'robot_type': P('robot_type'),
            'stand_height': P('stand_height'),
        }.items(),
    )

    traj_server = Node(
        package='plan_manage',
        executable='traj_server',
        name=f'traj_server_{drone_id}',
        output='screen',
        remappings=[
            ('/odom_world', odometry_topic),
            ('/planning/bspline', f'/planning/bspline_{drone_id}'),
            ('/planning/replan', f'/planning/replan_{drone_id}'),
            ('/planning/new', f'/planning/new_{drone_id}'),
            ('/position_cmd', f'planning/pos_cmd_{drone_id}'),
            ('planning/position_cmd_vis', f'planning/position_cmd_vis_{drone_id}'),
            ('planning/travel_traj', f'planning/travel_traj_{drone_id}'),
        ],
        parameters=[{
            'traj_server.time_forward': 1.5,
            'traj_server.pub_traj_id': 4,
            'traj_server.drone_id': int(drone_id),
            'traj_server.drone_num': int(drone_num),
            'perception_utils.top_angle': 0.56125,
            'perception_utils.left_angle': 0.69222,
            'perception_utils.right_angle': 0.68901,
            'perception_utils.max_dist': 4.5,
            'perception_utils.vis_dist': 1.0,
        }],
    )

    sim_type = P('sim_type')
    sim_launch_file = 'simulator_full.launch.py' if sim_type == 'full' else 'simulator.launch.py'
    simulator_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', sim_launch_file)),
        launch_arguments={
            'drone_id': drone_id,
            'drone_num': drone_num,
            'init_x': P('init_x'),
            'init_y': P('init_y'),
            'init_z': P('init_z'),
            'map_size_x': P('map_size_x'),
            'map_size_y': P('map_size_y'),
            'map_size_z': P('map_size_z'),
            'odometry_topic': odometry_topic,
        }.items(),
        condition=IfCondition(P('simulation')),
    )

    return [planner_include, traj_server, simulator_include]


def generate_launch_description():
    return LaunchDescription(_args() + [OpaqueFunction(function=_build)])
