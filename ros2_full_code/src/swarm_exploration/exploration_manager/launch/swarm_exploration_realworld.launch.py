"""Realworld (hardware) single-drone exploration launch.

ROS 1 equivalent: swarm_exploration_realworld.launch. Despite the name, the
ROS 1 file only spawns ONE drone — it expects operators to run this script on
each drone with a different drone_id. We preserve that behaviour.
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
        DeclareLaunchArgument('drone_num', default_value='4'),
        DeclareLaunchArgument('map_size_x', default_value='35.0'),
        DeclareLaunchArgument('map_size_y', default_value='25.0'),
        DeclareLaunchArgument('map_size_z', default_value='3.5'),
        DeclareLaunchArgument('odom_prefix', default_value='/state_ukf/odom'),
    ]


def _build(context):
    def P(n):
        return LaunchConfiguration(n).perform(context)

    share = get_package_share_directory('exploration_manager')
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, 'launch',
                         'single_drone_exploration_realworld.launch.py')),
        launch_arguments={
            'drone_id': P('drone_id'),
            'drone_num': P('drone_num'),
            'init_x': '0.0', 'init_y': '0.0', 'init_z': '0.0',
            'map_size_x': P('map_size_x'),
            'map_size_y': P('map_size_y'),
            'map_size_z': P('map_size_z'),
            'odom_prefix': P('odom_prefix'),
            'simulation': 'false',
            'odometry_topic': '/vins_estimator/odometry',
            'sensor_pose_topic': '/vins_estimator/camera_pose',
            'depth_topic': '/camera/depth/image_rect_raw',
            'cx': '326.34564209', 'cy': '239.099884033',
            'fx': '384.840637207', 'fy': '384.840637207',
        }.items(),
    )]


def generate_launch_description():
    return LaunchDescription(_args() + [OpaqueFunction(function=_build)])
