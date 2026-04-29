"""Standalone launcher for the so3_control composable node.

The ROS 2 port turned the ROS 1 nodelet into `so3_control::SO3ControlComponent`.
It still needs to run inside a container. This file wraps the component in a
single-node ComposableNodeContainer and loads both the Hummingbird gains and
corrections YAMLs installed alongside this package.
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os


def generate_launch_description():
    share = get_package_share_directory('so3_control')
    gains_yaml = os.path.join(share, 'config', 'gains_hummingbird.yaml')
    corrections_yaml = os.path.join(share, 'config', 'corrections_hummingbird.yaml')

    ns_arg = DeclareLaunchArgument('namespace', default_value='',
                                   description='ROS namespace for the controller.')

    container = ComposableNodeContainer(
        name='so3_control_container',
        namespace=LaunchConfiguration('namespace'),
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='so3_control',
                plugin='so3_control::SO3ControlComponent',
                name='so3_control',
                parameters=[
                    gains_yaml,
                    corrections_yaml,
                    {
                        'mass': 0.98,
                        'use_angle_corrections': False,
                        'use_external_yaw': False,
                        'gains.rot.z': 1.0,
                        'gains.ang.z': 0.1,
                    },
                ],
                remappings=[
                    ('~/odom', '/state_ukf/odom'),
                    ('~/position_cmd', '/planning/pos_cmd'),
                    ('~/motors', 'motors'),
                    ('~/corrections', 'corrections'),
                    ('~/so3_cmd', 'so3_cmd'),
                ],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([ns_arg, container])
