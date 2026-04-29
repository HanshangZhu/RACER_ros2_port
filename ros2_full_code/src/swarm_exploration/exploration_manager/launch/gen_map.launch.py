"""Map-generation helper (ROS 2 port of gen_map.launch).

ROS 1 equivalent spawned `map_generator/random_forest` with a big block of
`ObstacleShape`, `map/*` and `sensing/*` parameters to synthesise a random
cluttered world for planner experiments.

Runtime caveat: `random_forest` has **not** been ported to the ROS 2
map_generator package (the ROS 2 port currently ships `map_pub`, `click_map`,
`map_recorder`, `pilar_map`). This launch will therefore fail at spawn until
the executable is ported. The launch is kept as a mechanical mirror so that
re-enabling it later is just a matter of re-adding the executable.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    random_forest = Node(
        package='map_generator',
        executable='random_forest',
        name='random_forest',
        output='screen',
        remappings=[('~/odometry', 'empty')],
        parameters=[{
            'init_state_x': 0.0,
            'init_state_y': 0.0,
            'map.x_size': 20.0,
            'map.y_size': 20.0,
            'map.z_size': 3.0,
            'map.resolution': 0.1,

            'ObstacleShape.seed': 1,
            'map.obs_num': 0,
            'map.circle_num': 0,

            'ObstacleShape.lower_rad': 0.5,
            'ObstacleShape.upper_rad': 0.8,
            'ObstacleShape.lower_hei': 0.0,
            'ObstacleShape.upper_hei': 3.0,

            'ObstacleShape.radius_l': 0.7,
            'ObstacleShape.radius_h': 0.8,
            'ObstacleShape.z_l': 0.7,
            'ObstacleShape.z_h': 0.8,
            'ObstacleShape.theta': 0.5,

            'sensing.radius': 5.0,
            'sensing.rate': 10.0,
        }],
    )

    return LaunchDescription([random_forest])
