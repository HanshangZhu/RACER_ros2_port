"""Real-hardware launch for a single Unitree Go1 / Go2 running RACER.

Runs the planner + traj_server + LKH solver + Twist bridge, and subscribes
to the robot's real odometry and depth topics. Does NOT start the simulator,
pcl_render_node, or map_pub (those are sim-only).

Prereqs (see docs/go2_deployment.md for full setup):
  * unitree_ros2 / go2_ros2_sdk running on the robot's onboard NUC or the host PC.
  * Odometry published on `/utlidar/robot_pose` or `/odom` (configurable via odom_topic).
  * Depth image on `/camera/depth/image_rect_raw` from an onboard D435 (or L1 LiDAR if
    you flip the launch to use cloud_topic instead).

Run with e.g.:
    ros2 launch exploration_manager real_go2.launch.py drone_id:=1

This publishes `/cmd_vel` which the Go2 SDK consumes directly.
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
        DeclareLaunchArgument('map_size_x', default_value='20.0'),
        DeclareLaunchArgument('map_size_y', default_value='20.0'),
        DeclareLaunchArgument('map_size_z', default_value='2.0'),
        DeclareLaunchArgument('stand_height', default_value='0.30',
                              description='Go1 ~0.32, Go2 ~0.30 (calibrate).'),
        # Go2 SDK topics — override if your topology differs.
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('sensor_pose_topic',
                              default_value='/camera/pose'),
        DeclareLaunchArgument('depth_topic',
                              default_value='/camera/depth/image_rect_raw'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        # Intel RealSense D435 on Go2 default intrinsics — check with your own calibration.
        DeclareLaunchArgument('cx', default_value='326.34564209'),
        DeclareLaunchArgument('cy', default_value='239.099884033'),
        DeclareLaunchArgument('fx', default_value='384.840637207'),
        DeclareLaunchArgument('fy', default_value='384.840637207'),
        DeclareLaunchArgument('v_fwd_max', default_value='1.0'),
        DeclareLaunchArgument('v_lat_max', default_value='0.5'),
        DeclareLaunchArgument('omega_max', default_value='1.0'),
    ]


def _build(context):
    def P(n):
        return LaunchConfiguration(n).perform(context)

    drone_id = P('drone_id')
    drone_num = P('drone_num')
    odom_topic = P('odom_topic')
    share = get_package_share_directory('exploration_manager')

    planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'single_drone_planner.launch.py')),
        launch_arguments={
            'drone_id': drone_id,
            'drone_num': drone_num,
            'map_size_x': P('map_size_x'),
            'map_size_y': P('map_size_y'),
            'map_size_z': P('map_size_z'),
            'odometry_topic': odom_topic,
            'sensor_pose_topic': P('sensor_pose_topic'),
            'depth_topic': P('depth_topic'),
            'cloud_topic': f'/pcl_render_node/cloud_{drone_id}',  # unused on real robot
            'cx': P('cx'), 'cy': P('cy'), 'fx': P('fx'), 'fy': P('fy'),
            'max_vel': '1.0', 'max_acc': '0.5',
            'simulation': 'false',
            'robot_type': 'quadruped',
            'stand_height': P('stand_height'),
        }.items(),
    )

    traj_server = Node(
        package='plan_manage', executable='traj_server',
        name=f'traj_server_{drone_id}', output='screen',
        remappings=[
            ('/odom_world', odom_topic),
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

    twist_bridge = Node(
        package='quadruped_simulator', executable='cmd_to_twist',
        name=f'cmd_to_twist_{drone_id}', output='screen',
        parameters=[{
            'v_fwd_max': float(P('v_fwd_max')),
            'v_lat_max': float(P('v_lat_max')),
            'omega_max': float(P('omega_max')),
            'rate_hz': 50.0,
        }],
        remappings=[
            ('command', f'/planning/pos_cmd_{drone_id}'),
            ('odometry', odom_topic),
            ('cmd_vel', P('cmd_vel_topic')),
        ],
    )

    tsp_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'tsp_server.launch.py')),
        launch_arguments={'drone_num': drone_num}.items(),
    )

    return [planner, traj_server, twist_bridge, tsp_server]


def generate_launch_description():
    return LaunchDescription(_args() + [OpaqueFunction(function=_build)])
