"""Optional centralised aggregator (one ground_node per swarm).

ROS 1 equivalent: ground_node.launch. Spawns a single `ground_node` executable
(from exploration_manager) responsible for fusing maps/frontiers across
drones. Param names are dot-separated to match the ROS 2 C++ port.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    drone_id_arg = DeclareLaunchArgument('drone_id', default_value='6')
    drone_num_arg = DeclareLaunchArgument('drone_num', default_value='6')
    vis_drone_id_arg = DeclareLaunchArgument('vis_drone_id', default_value='1')

    map_size_x_arg = DeclareLaunchArgument('map_size_x', default_value='35.0')
    map_size_y_arg = DeclareLaunchArgument('map_size_y', default_value='25.0')
    map_size_z_arg = DeclareLaunchArgument('map_size_z', default_value='3.5')

    drone_id = LaunchConfiguration('drone_id')

    # All per-drone topic remaps use $(arg drone_id); in ROS 2 we match with
    # substitutions in the remappings list.
    ns_suffix = ['_', drone_id]

    def suffix(prefix):
        return [prefix] + ns_suffix

    ground_node = Node(
        package='exploration_manager',
        executable='ground_node',
        name=['ground_node_', drone_id],
        output='screen',
        remappings=[
            ('/sdf_map/occupancy_all', suffix('/sdf_map/occupancy_all')),
            ('/sdf_map/occupancy_local', suffix('/sdf_map/occupancy_local')),
            ('/sdf_map/occupancy_local_inflate', suffix('/sdf_map/occupancy_local_inflate')),
            ('/sdf_map/update_range', suffix('/sdf_map/update_range')),
            ('/multi_map_manager/chunk_stamps_send', '/multi_map_manager/chunk_stamps'),
            ('/multi_map_manager/chunk_data_send', '/multi_map_manager/chunk_data'),
            ('/multi_map_manager/chunk_stamps_recv', '/multi_map_manager/chunk_stamps'),
            ('/multi_map_manager/chunk_data_recv', '/multi_map_manager/chunk_data'),
        ],
        parameters=[{
            'sdf_map.resolution': 0.1,
            'sdf_map.map_size_x': LaunchConfiguration('map_size_x'),
            'sdf_map.map_size_y': LaunchConfiguration('map_size_y'),
            'sdf_map.map_size_z': LaunchConfiguration('map_size_z'),
            'sdf_map.obstacles_inflation': 0.199,
            'sdf_map.local_bound_inflate': 0.5,
            'sdf_map.local_map_margin': 50,
            'sdf_map.ground_height': -1.0,
            'sdf_map.default_dist': 0.0,

            'sdf_map.p_hit': 0.65,
            'sdf_map.p_miss': 0.35,
            'sdf_map.p_min': 0.12,
            'sdf_map.p_max': 0.90,
            'sdf_map.p_occ': 0.80,
            'sdf_map.min_ray_length': 0.5,
            'sdf_map.max_ray_length': 4.5,
            'sdf_map.virtual_ceil_height': -10.0,
            'sdf_map.optimistic': False,
            'sdf_map.signed_dist': False,
            'sdf_map.box_min_x': -10.0,
            'sdf_map.box_min_y': -10.0,
            'sdf_map.box_min_z': 0.0,
            'sdf_map.box_max_x': 10.0,
            'sdf_map.box_max_y': 10.0,
            'sdf_map.box_max_z': 2.0,

            'map_ros.depth_filter_maxdist': 5.0,
            'map_ros.depth_filter_mindist': 0.2,
            'map_ros.depth_filter_margin': 2,
            'map_ros.k_depth_scaling_factor': 1000.0,
            'map_ros.skip_pixel': 2,
            'map_ros.esdf_slice_height': 0.3,
            'map_ros.visualization_truncate_height': 10.09,
            'map_ros.visualization_truncate_low': -2.0,
            'map_ros.show_occ_time': False,
            'map_ros.show_esdf_time': False,
            'map_ros.show_all_map': True,
            'map_ros.frame_id': 'world',

            'exploration.drone_num': LaunchConfiguration('drone_num'),
            'exploration.drone_id': drone_id,
            'exploration.vis_drone_id': LaunchConfiguration('vis_drone_id'),

            'manager.use_geometric_path': True,
            'astar.lambda_heu': 10000.0,
            'astar.resolution_astar': 0.2,
            'astar.allocate_num': 1000000,
            'astar.max_search_time': 0.005,

            'frontier.cluster_min': 100,
            'frontier.cluster_size_xy': 2.0,
            'frontier.cluster_size_z': 10.0,
            'frontier.min_candidate_dist': 0.5,
            'frontier.min_candidate_clearance': 0.21,
            'frontier.candidate_dphi': 15.0 * 3.1415926 / 180.0,
            'frontier.candidate_rnum': 3,
            'frontier.candidate_rmin': 1.0,
            'frontier.candidate_rmax': 1.5,
            'frontier.down_sample': 3,
            'frontier.min_visib_num': 30,
            'frontier.min_view_finish_fraction': 0.2,

            'perception_utils.top_angle': 0.56125,
            'perception_utils.left_angle': 0.69222,
            'perception_utils.right_angle': 0.68901,
            'perception_utils.max_dist': 4.5,
            'perception_utils.vis_dist': 1.0,
        }],
    )

    return LaunchDescription([
        drone_id_arg,
        drone_num_arg,
        vis_drone_id_arg,
        map_size_x_arg,
        map_size_y_arg,
        map_size_z_arg,
        ground_node,
    ])
