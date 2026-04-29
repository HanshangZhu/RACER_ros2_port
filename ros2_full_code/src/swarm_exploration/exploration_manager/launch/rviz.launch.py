"""Launch rviz2 with the patched RACER exploration config.

ROS 1 equivalent: rviz.launch (single `<node pkg="rviz" type="rviz" args="-d ...swarm.rviz">`).
The .rviz file has been translated to ROS 2 display/panel class names and lives
at `exploration_manager/resource/exploration.rviz`.
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('exploration_manager')

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([share, 'resource', 'exploration.rviz']),
        description='Path to the .rviz config file.',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rvizvisualisation',
        output='log',
        arguments=['-d', LaunchConfiguration('rviz_config')],
    )

    return LaunchDescription([rviz_config_arg, rviz])
