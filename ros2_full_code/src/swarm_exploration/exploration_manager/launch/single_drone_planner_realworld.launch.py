"""Per-drone realworld planner launch.

ROS 1 equivalent: single_drone_planner_realworld.xml. This is a thin wrapper
around single_drone_planner.launch.py with `realworld:=true`, which flips the
planner into hardware-deployment tuning (tighter map box, alternate SDF probs,
per-drone swarm_expl/* topic suffixes, lower replan cadence).
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os


def _args():
    return [
        DeclareLaunchArgument('drone_id', default_value='1'),
        DeclareLaunchArgument('drone_num', default_value='1'),
        DeclareLaunchArgument('map_size_x'),
        DeclareLaunchArgument('map_size_y'),
        DeclareLaunchArgument('map_size_z'),
        DeclareLaunchArgument('odometry_topic'),
        DeclareLaunchArgument('sensor_pose_topic'),
        DeclareLaunchArgument('depth_topic'),
        DeclareLaunchArgument('cloud_topic'),
        DeclareLaunchArgument('cx'), DeclareLaunchArgument('cy'),
        DeclareLaunchArgument('fx'), DeclareLaunchArgument('fy'),
        DeclareLaunchArgument('max_vel', default_value='1.0'),
        DeclareLaunchArgument('max_acc', default_value='0.8'),
        DeclareLaunchArgument('simulation', default_value='false'),
        DeclareLaunchArgument('single_expo', default_value='false'),
    ]


def _build(context):
    def P(n):
        return LaunchConfiguration(n).perform(context)

    share = get_package_share_directory('exploration_manager')
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'single_drone_planner.launch.py')),
        launch_arguments={
            'drone_id': P('drone_id'), 'drone_num': P('drone_num'),
            'map_size_x': P('map_size_x'), 'map_size_y': P('map_size_y'),
            'map_size_z': P('map_size_z'),
            'odometry_topic': P('odometry_topic'),
            'sensor_pose_topic': P('sensor_pose_topic'),
            'depth_topic': P('depth_topic'), 'cloud_topic': P('cloud_topic'),
            'cx': P('cx'), 'cy': P('cy'), 'fx': P('fx'), 'fy': P('fy'),
            'max_vel': P('max_vel'), 'max_acc': P('max_acc'),
            'simulation': P('simulation'),
            'realworld': 'true',
            'single_expo': P('single_expo'),
        }.items(),
    )]


def generate_launch_description():
    return LaunchDescription(_args() + [OpaqueFunction(function=_build)])
