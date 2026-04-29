"""RViz + rosbag replay helper (ROS 2 port of bag_rviz.launch).

ROS 1 equivalent was a 3-line file that launched rviz with
`$(find exploration_manager)/config/swarm_bag.rviz`. That legacy .rviz file
has **not** been translated to rviz2 format yet; we point at the already-
ported `exploration.rviz` by default so the launch succeeds. Override
`rviz_config` on the command line if you have a dedicated bag-replay .rviz.

TODO (rosbag): ROS 1 `<node pkg="rosbag" type="play" ...>` has no direct ROS 2
equivalent in launch files. Run `ros2 bag play <bag>` in a separate terminal.
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
        description='Path to the .rviz config file (rviz2 YAML).',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rvizvisualisation',
        output='log',
        arguments=['-d', LaunchConfiguration('rviz_config')],
    )

    return LaunchDescription([rviz_config_arg, rviz])
