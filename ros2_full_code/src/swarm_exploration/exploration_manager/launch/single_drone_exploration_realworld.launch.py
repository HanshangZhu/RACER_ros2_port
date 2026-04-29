"""Per-drone realworld exploration stack (planner + traj_server, no simulator).

ROS 1 equivalent: single_drone_exploration_realworld.xml.
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def _args():
    return [
        DeclareLaunchArgument('drone_id', default_value='1'),
        DeclareLaunchArgument('drone_num', default_value='1'),
        DeclareLaunchArgument('init_x', default_value='0.0'),
        DeclareLaunchArgument('init_y', default_value='0.0'),
        DeclareLaunchArgument('init_z', default_value='0.0'),
        DeclareLaunchArgument('map_size_x', default_value='22.0'),
        DeclareLaunchArgument('map_size_y', default_value='22.0'),
        DeclareLaunchArgument('map_size_z', default_value='3.5'),
        DeclareLaunchArgument('odom_prefix', default_value='odom'),
        DeclareLaunchArgument('simulation', default_value='false'),
        DeclareLaunchArgument('odometry_topic',
                              default_value='/vins_estimator/odometry'),
        DeclareLaunchArgument('sensor_pose_topic',
                              default_value='/vins_estimator/camera_pose'),
        DeclareLaunchArgument('depth_topic',
                              default_value='/camera/depth/image_rect_raw'),
        DeclareLaunchArgument('cx', default_value='326.34564209'),
        DeclareLaunchArgument('cy', default_value='239.099884033'),
        DeclareLaunchArgument('fx', default_value='384.840637207'),
        DeclareLaunchArgument('fy', default_value='384.840637207'),
    ]


def _build(context):
    def P(n):
        return LaunchConfiguration(n).perform(context)

    drone_id = P('drone_id')
    drone_num = P('drone_num')
    odom_prefix = P('odom_prefix')
    share = get_package_share_directory('exploration_manager')

    planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'single_drone_planner_realworld.launch.py')),
        launch_arguments={
            'drone_id': drone_id, 'drone_num': drone_num,
            'map_size_x': P('map_size_x'), 'map_size_y': P('map_size_y'),
            'map_size_z': P('map_size_z'),
            'odometry_topic': P('odometry_topic'),
            'sensor_pose_topic': P('sensor_pose_topic'),
            'depth_topic': P('depth_topic'),
            'cloud_topic': f'/pcl_render_node/cloud_{drone_id}',
            'cx': P('cx'), 'cy': P('cy'), 'fx': P('fx'), 'fy': P('fy'),
            'max_vel': '1.0', 'max_acc': '0.8',
            'simulation': P('simulation'),
        }.items(),
    )

    traj_server = Node(
        package='plan_manage',
        executable='traj_server',
        name=f'traj_server_{drone_id}',
        output='screen',
        remappings=[
            ('/odom_world', f'{odom_prefix}_{drone_id}'),
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
            'traj_server.init_x': float(P('init_x')),
            'traj_server.init_y': float(P('init_y')),
            'perception_utils.top_angle': 0.56125,
            'perception_utils.left_angle': 0.69222,
            'perception_utils.right_angle': 0.68901,
            'perception_utils.max_dist': 4.5,
            'perception_utils.vis_dist': 1.0,
        }],
    )

    return [planner, traj_server]


def generate_launch_description():
    return LaunchDescription(_args() + [OpaqueFunction(function=_build)])
