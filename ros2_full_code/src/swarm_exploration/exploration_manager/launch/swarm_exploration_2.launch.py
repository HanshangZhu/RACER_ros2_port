"""Alternative swarm exploration (ROS 2 port of swarm_exploration_2.launch).

ROS 1 equivalent spawned 2 active drones (ids 1 and 2) with drone_num=3 on the
`explore1.pcd` environment (35x25x3.5 m), plus the `local_sensing_node/sim_swarm_tf`
swarm-TF helper.

Runtime caveat: `sim_swarm_tf` is a ROS 1 executable from the `local_sensing`
stack that has not been ported to the ROS 2 `local_sensing` package yet.
The Node is included mechanically and will fail at spawn until the executable
is ported.
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os


# (drone_id, init_x, init_y, init_z) — only 1 and 2 are active in the ROS 1 XML.
DRONES = [
    (1, -8.0, 1.5, 1.0),
    (2, -8.0, 0.5, 1.0),
]

MAP_SIZE = ('35.0', '25.0', '3.5')
ODOM_PREFIX = '/state_ukf/odom'
DRONE_NUM = '3'
PCD_NAME = 'explore1.pcd'


def generate_launch_description():
    exploration_share = get_package_share_directory('exploration_manager')
    map_generator_share = get_package_share_directory('map_generator')

    actions = []

    # Map publisher (explore1.pcd) ------------------------------------------
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(map_generator_share, 'launch', 'map_pub.launch.py')),
        launch_arguments={'pcd_name': PCD_NAME}.items(),
    ))

    # Per-drone exploration stacks ------------------------------------------
    for drone_id, x, y, z in DRONES:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(exploration_share, 'launch',
                             'single_drone_exploration.launch.py')),
            launch_arguments={
                'drone_id': str(drone_id),
                'drone_num': DRONE_NUM,
                'init_x': str(x),
                'init_y': str(y),
                'init_z': str(z),
                'map_size_x': MAP_SIZE[0],
                'map_size_y': MAP_SIZE[1],
                'map_size_z': MAP_SIZE[2],
                'odom_prefix': ODOM_PREFIX,
                'simulation': 'true',
            }.items(),
        ))

    # Swarm TF helper (ROS 1 `local_sensing_node/sim_swarm_tf`). See file
    # docstring for the porting caveat.
    actions.append(Node(
        package='local_sensing',
        executable='sim_swarm_tf',
        name='sim_swarm_tf',
        output='screen',
        parameters=[{'drone_num': int(DRONE_NUM)}],
    ))

    # LKH TSP/MTSP services --------------------------------------------------
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(exploration_share, 'launch', 'tsp_server.launch.py')),
        launch_arguments={'drone_num': DRONE_NUM}.items(),
    ))

    return LaunchDescription(actions)
