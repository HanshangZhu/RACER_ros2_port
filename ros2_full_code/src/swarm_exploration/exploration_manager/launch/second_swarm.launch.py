"""Alternative 'second swarm' launch (ROS 2 port of second_swarm.launch).

ROS 1 equivalent spawned a single drone (drone_id=3, init at (-3,-1,1)) with
drone_num=4 (so it participates as drone 3 of a 4-drone swarm whose peers
run elsewhere). Map publisher is intentionally **not** started — the other
launch is expected to own it.
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    exploration_share = get_package_share_directory('exploration_manager')

    single_drone = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(exploration_share, 'launch',
                         'single_drone_exploration.launch.py')),
        launch_arguments={
            'drone_id': '3',
            'drone_num': '4',
            'init_x': '-3.0',
            'init_y': '-1.0',
            'init_z': '1.0',
            'map_size_x': '35.0',
            'map_size_y': '25.0',
            'map_size_z': '3.5',
            'odom_prefix': '/state_ukf/odom',
            'simulation': 'true',
        }.items(),
    )

    return LaunchDescription([single_drone])
