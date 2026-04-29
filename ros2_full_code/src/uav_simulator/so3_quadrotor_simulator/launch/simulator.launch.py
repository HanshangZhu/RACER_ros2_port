"""Standalone so3_quadrotor_simulator demo (sim + controller + disturbance + rviz2).

ROS 1 equivalent: so3_quadrotor_simulator/launch/simulator.launch. This is
NOT the swarm demo — it brings up a single quadrotor under the full SO(3)
physics stack with its own rviz2 config, useful for controller-only
experiments. The swarm variant is simulator_full.launch.py in
exploration_manager.
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
import os


def generate_launch_description():
    so3_share = get_package_share_directory('so3_control')
    sim_share = get_package_share_directory('so3_quadrotor_simulator')
    gains_yaml = os.path.join(so3_share, 'config', 'gains_hummingbird.yaml')
    corrections_yaml = os.path.join(so3_share, 'config', 'corrections_hummingbird.yaml')
    rviz_config = os.path.join(sim_share, 'config', 'rviz.rviz')

    quadrotor = Node(
        package='so3_quadrotor_simulator',
        executable='quadrotor_simulator_so3',
        name='quadrotor_simulator_so3',
        output='screen',
        parameters=[{
            'rate.odom': 100.0,
            'simulator.init_state_x': -5.0,
            'simulator.init_state_y': 0.0,
            'simulator.init_state_z': 3.0,
        }],
        remappings=[
            ('odom', '/visual_slam/odom'),
            ('cmd', 'so3_cmd'),
            ('imu', 'sim/imu'),
            ('force_disturbance', 'force_disturbance'),
            ('moment_disturbance', 'moment_disturbance'),
        ],
    )

    so3_control = ComposableNodeContainer(
        name='so3_control_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        output='screen',
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
                    ('odom', '/state_ukf/odom'),
                    ('position_cmd', 'position_cmd'),
                    ('motors', 'motors'),
                    ('corrections', 'corrections'),
                    ('so3_cmd', 'so3_cmd'),
                    ('imu', 'sim/imu'),
                ],
            ),
        ],
    )

    disturbance = Node(
        package='so3_disturbance_generator',
        executable='so3_disturbance_generator',
        name='so3_disturbance_generator',
        output='screen',
        remappings=[
            ('odom', '/visual_slam/odom'),
            ('noisy_odom', '/state_ukf/odom'),
            ('correction', '/visual_slam/correction'),
            ('force_disturbance', 'force_disturbance'),
            ('moment_disturbance', 'moment_disturbance'),
        ],
    )

    odom_vis = Node(
        package='odom_visualization',
        executable='odom_visualization',
        name='odom_visualization_ukf',
        output='screen',
        parameters=[{
            'color.a': 0.8,
            'color.r': 1.0,
            'color.g': 0.0,
            'color.b': 0.0,
            'covariance_scale': 100.0,
        }],
        remappings=[('~/odom', '/visual_slam/odom')],
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([quadrotor, so3_control, disturbance, odom_vis, rviz])
