"""Map-publisher launch for the RACER ROS 2 port.

ROS 1 equivalent: the single `<node pkg="map_generator" type="map_pub" ... args=".../pillar.pcd">`
block inside `swarm_exploration.launch`. Centralising it here lets the swarm
launch (and any future launch) include a single consistent map publisher.
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    map_generator_share = get_package_share_directory('map_generator')

    pcd_name_arg = DeclareLaunchArgument(
        'pcd_name',
        default_value='pillar.pcd',
        description='Filename of the .pcd environment under map_generator/resource/.',
    )

    pcd_path = PathJoinSubstitution([
        map_generator_share, 'resource', LaunchConfiguration('pcd_name')
    ])

    map_pub = Node(
        package='map_generator',
        executable='map_pub',
        name='map_pub',
        output='screen',
        # map_publisher.cpp takes the pcd file as argv[1].
        arguments=[pcd_path],
    )

    return LaunchDescription([pcd_name_arg, map_pub])
