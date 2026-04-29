"""Top-level 4-drone swarm exploration demo (ROS 2 port).

ROS 1 equivalent: swarm_exploration.launch. The original XML hand-unrolls 4
identical `<include>` blocks (one per drone); here we collapse that into a
single Python `for` loop. Change `num_drones` (launch arg) or edit the
`DEFAULT_POSITIONS` table below to add / reshuffle drones.

Starts:
  * 1 map_pub (loads pillar.pcd by default)
  * `num_drones` exploration stacks (planner + traj_server + simulator)
  * LKH TSP/MTSP service nodes (via tsp_server.launch.py)

Companion launches (not started here — run separately):
  * rviz.launch.py            - visualisation
  * ground_node.launch.py     - optional centralised map aggregator
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os


# Drone initial positions copied from the ROS 1 swarm_exploration.launch XML
# (index 0 unused; drone ids are 1-based). Add entries to grow the swarm.
DEFAULT_POSITIONS = {
    1: (0.0, 0.0, 1.0),
    2: (0.0, -1.0, 1.0),
    3: (1.0, -1.0, 1.0),
    4: (1.0, 0.0, 1.0),
    5: (1.0, 1.0, 1.0),
    6: (0.0, 1.0, 1.0),
    7: (-1.0, 1.0, 1.0),
    8: (-1.0, 0.0, 1.0),
    9: (-1.0, -1.0, 1.0),
    10: (0.0, -5.0, 1.0),
}


def _args():
    return [
        DeclareLaunchArgument('num_drones', default_value='4',
                              description='Number of drones to spawn (1..10).'),
        DeclareLaunchArgument('map_size_x', default_value='35.0'),
        DeclareLaunchArgument('map_size_y', default_value='35.0'),
        DeclareLaunchArgument('map_size_z', default_value='3.5'),
        DeclareLaunchArgument('odom_prefix', default_value='/state_ukf/odom'),
        DeclareLaunchArgument('pcd_name', default_value='pillar.pcd',
                              description='Environment .pcd (under map_generator/resource/).'),
        DeclareLaunchArgument('simulation', default_value='true'),
        DeclareLaunchArgument('sim_type', default_value='light',
                              description='light (cmd2odom, matches ROS 1 demo) | '
                                          'full (heavy SO3 physics + controller).'),
        DeclareLaunchArgument('robot_type', default_value='quadrotor',
                              description='quadrotor | quadruped. Quadruped swaps the '
                                          'light sim for quadruped_simulator/cmd2base, '
                                          'pins z to stand_height, tightens velocity caps.'),
        DeclareLaunchArgument('stand_height', default_value='0.30'),
    ]


def _build(context):
    num_drones = int(LaunchConfiguration('num_drones').perform(context))
    map_size_x = LaunchConfiguration('map_size_x').perform(context)
    map_size_y = LaunchConfiguration('map_size_y').perform(context)
    map_size_z = LaunchConfiguration('map_size_z').perform(context)
    odom_prefix = LaunchConfiguration('odom_prefix').perform(context)
    pcd_name = LaunchConfiguration('pcd_name').perform(context)
    simulation = LaunchConfiguration('simulation').perform(context)
    sim_type = LaunchConfiguration('sim_type').perform(context)
    robot_type = LaunchConfiguration('robot_type').perform(context)
    stand_height = LaunchConfiguration('stand_height').perform(context)

    if num_drones not in range(1, len(DEFAULT_POSITIONS) + 1):
        raise RuntimeError(
            f'num_drones={num_drones} out of range (1..{len(DEFAULT_POSITIONS)}). '
            'Add more entries to DEFAULT_POSITIONS to support larger swarms.')

    exploration_share = get_package_share_directory('exploration_manager')
    map_generator_share = get_package_share_directory('map_generator')

    actions = []

    # 1) Map publisher -------------------------------------------------------
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(map_generator_share, 'launch', 'map_pub.launch.py')),
        launch_arguments={'pcd_name': pcd_name}.items(),
    ))

    # 2) Per-drone exploration stack (collapsed from 4x copy-paste in XML) --
    for i in range(1, num_drones + 1):
        x, y, z = DEFAULT_POSITIONS[i]
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(exploration_share, 'launch',
                             'single_drone_exploration.launch.py')),
            launch_arguments={
                'drone_id': str(i),
                'drone_num': str(num_drones),
                'init_x': str(x),
                'init_y': str(y),
                'init_z': str(z),
                'map_size_x': map_size_x,
                'map_size_y': map_size_y,
                'map_size_z': map_size_z,
                'odom_prefix': odom_prefix,
                'simulation': simulation,
                'sim_type': sim_type,
                'robot_type': robot_type,
                'stand_height': stand_height,
            }.items(),
        ))

    # 3) LKH TSP/MTSP service servers ---------------------------------------
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(exploration_share, 'launch', 'tsp_server.launch.py')),
        launch_arguments={'drone_num': str(num_drones)}.items(),
    ))

    return actions


def generate_launch_description():
    return LaunchDescription(_args() + [OpaqueFunction(function=_build)])
