"""Standalone traj_server launcher (one drone).

Useful when debugging the trajectory server in isolation. The swarm demo
invokes traj_server directly from single_drone_exploration.launch.py.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    drone_id_arg = DeclareLaunchArgument('drone_id', default_value='1')
    drone_num_arg = DeclareLaunchArgument('drone_num', default_value='1')

    drone_id = LaunchConfiguration('drone_id')
    drone_num = LaunchConfiguration('drone_num')

    traj_server = Node(
        package='plan_manage',
        executable='traj_server',
        name=['traj_server_', drone_id],
        output='screen',
        parameters=[{
            'traj_server.time_forward': 1.5,
            'traj_server.pub_traj_id': 4,
            'traj_server.drone_id': drone_id,
            'traj_server.drone_num': drone_num,
            'perception_utils.top_angle': 0.56125,
            'perception_utils.left_angle': 0.69222,
            'perception_utils.right_angle': 0.68901,
            'perception_utils.max_dist': 4.5,
            'perception_utils.vis_dist': 1.0,
        }],
    )

    return LaunchDescription([drone_id_arg, drone_num_arg, traj_server])
