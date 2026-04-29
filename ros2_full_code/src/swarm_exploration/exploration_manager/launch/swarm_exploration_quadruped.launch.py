"""Quadruped-flavoured swarm exploration demo.

Thin wrapper over swarm_exploration.launch.py that sets `robot_type:=quadruped`
and `sim_type:=light` (quadruped variant only has a light sim — the heavy SO3
physics stack doesn't apply to legged robots). Keeps `num_drones` / `pcd_name`
overridable for parity with the quadrotor demo.
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
        DeclareLaunchArgument('num_drones', default_value='1',
                              description='Number of quadrupeds to spawn.'),
        DeclareLaunchArgument('pcd_name', default_value='pillar.pcd'),
        DeclareLaunchArgument('stand_height', default_value='0.30'),
        DeclareLaunchArgument('map_size_x', default_value='35.0'),
        DeclareLaunchArgument('map_size_y', default_value='35.0'),
        DeclareLaunchArgument('map_size_z', default_value='3.5'),
    ]


def _build(context):
    def P(n):
        return LaunchConfiguration(n).perform(context)

    share = get_package_share_directory('exploration_manager')
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'swarm_exploration.launch.py')),
        launch_arguments={
            'num_drones': P('num_drones'),
            'pcd_name': P('pcd_name'),
            'map_size_x': P('map_size_x'),
            'map_size_y': P('map_size_y'),
            'map_size_z': P('map_size_z'),
            'robot_type': 'quadruped',
            'sim_type': 'light',
            'stand_height': P('stand_height'),
        }.items(),
    )]


def generate_launch_description():
    return LaunchDescription(_args() + [OpaqueFunction(function=_build)])
