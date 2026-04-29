"""Standalone LKH TSP / MTSP service-node launcher.

ROS 1 equivalent: tsp_server.launch. One tsp_node per drone (single salesman)
plus one mtsp_node per drone (ACVRP). The ROS 1 file hard-codes 3 drones; we
parametrise `drone_num` and loop.

Exe names (verified under install/): `lkh_tsp_solver/lib/lkh_tsp_solver/tsp_node`
and `lkh_mtsp_solver/lib/lkh_mtsp_solver/mtsp_node`.
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    drone_num_arg = DeclareLaunchArgument(
        'drone_num',
        default_value='4',
        description='Number of per-drone TSP/MTSP solver pairs to spawn.',
    )

    tsp_dir = get_package_share_directory('lkh_tsp_solver') + '/resource'
    mtsp_dir = get_package_share_directory('lkh_mtsp_solver') + '/resource'

    def create_nodes(context):
        n = int(LaunchConfiguration('drone_num').perform(context))
        actions = []
        for i in range(1, n + 1):
            actions.append(Node(
                package='lkh_tsp_solver',
                executable='tsp_node',
                name=f'tsp_solver_{i}',
                output='screen',
                parameters=[{
                    'exploration.drone_id': i,
                    'exploration.tsp_dir': tsp_dir,
                }],
            ))
            actions.append(Node(
                package='lkh_mtsp_solver',
                executable='mtsp_node',
                name=f'mtsp_solver_{i}',
                output='screen',
                parameters=[{
                    'exploration.drone_id': i,
                    'exploration.mtsp_dir': mtsp_dir,
                    'exploration.problem_id': 1,
                }],
            ))
        return actions

    from launch.actions import OpaqueFunction
    return LaunchDescription([
        drone_num_arg,
        OpaqueFunction(function=create_nodes),
    ])
