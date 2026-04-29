"""Per-drone exploration_node parameters + remappings.

ROS 1 equivalent: single_drone_planner.xml. Produces one `exploration_node`
with ~150 dot-separated parameters and the full set of topic remaps that
namespace per-drone streams (sdf_map, planning, swarm_expl, ...).

Also spawns the per-drone MTSP + ACVRP solver nodes (equivalent to the
trailing `<node pkg="lkh_mtsp_solver"...>` blocks).
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import math


def _args():
    return [
        DeclareLaunchArgument('drone_id', default_value='1'),
        DeclareLaunchArgument('drone_num', default_value='4'),
        DeclareLaunchArgument('map_size_x', default_value='35.0'),
        DeclareLaunchArgument('map_size_y', default_value='35.0'),
        DeclareLaunchArgument('map_size_z', default_value='3.5'),
        DeclareLaunchArgument('odometry_topic', default_value='/state_ukf/odom_1'),
        DeclareLaunchArgument('sensor_pose_topic',
                              default_value='/pcl_render_node/sensor_pose_1'),
        DeclareLaunchArgument('depth_topic', default_value='/pcl_render_node/depth_1'),
        DeclareLaunchArgument('cloud_topic', default_value='/pcl_render_node/cloud_1'),
        DeclareLaunchArgument('cx', default_value='324.0879821777344'),
        DeclareLaunchArgument('cy', default_value='239.10362243652344'),
        DeclareLaunchArgument('fx', default_value='385.69793701171875'),
        DeclareLaunchArgument('fy', default_value='385.69793701171875'),
        DeclareLaunchArgument('max_vel', default_value='1.5'),
        DeclareLaunchArgument('max_acc', default_value='1.0'),
        DeclareLaunchArgument('simulation', default_value='true'),
        DeclareLaunchArgument('realworld', default_value='false',
                              description='Use real-world tuning (per-drone swarm_expl/* '
                                          'remaps, tighter map box, alternate SDF probs).'),
        DeclareLaunchArgument('single_expo', default_value='false',
                              description='realworld only: sdf_map.no_drone_1 flag.'),
        DeclareLaunchArgument('robot_type', default_value='quadrotor',
                              description='quadrotor (default) | quadruped — applies a '
                                          'quadruped-shaped tuning overlay (thin z-slab, '
                                          'tighter velocity caps, lower yaw rate).'),
        DeclareLaunchArgument('stand_height', default_value='0.30',
                              description='quadruped: base-link height (m) above ground.'),
    ]


def _build(context):
    def P(name):
        return LaunchConfiguration(name).perform(context)

    drone_id = P('drone_id')
    drone_num = P('drone_num')
    sim = P('simulation').lower() == 'true'
    realworld = P('realworld').lower() == 'true'
    single_expo = P('single_expo').lower() == 'true'
    quadruped = P('robot_type').lower() == 'quadruped'
    stand_height = float(P('stand_height'))

    # Per-drone topic suffix, matching the ROS 1 XML's `$(arg drone_id)` usage.
    s = lambda base: f'{base}_{drone_id}'  # noqa: E731

    # Resource dirs baked into the share of LKH packages.
    tsp_dir = get_package_share_directory('lkh_tsp_solver') + '/resource'
    mtsp_dir = get_package_share_directory('lkh_mtsp_solver') + '/resource'

    # The two basecoor remaps differ based on `simulation`; replicate both
    # branches from the XML via an if-check (ROS 2 has no <remap if=...>).
    basecoor_target = (
        f'/swarm_sim_tf/basecoor_{drone_id}' if sim else '/swarm_drones/swarm_drone_basecoor'
    )

    # swarm_expl / multi_map_manager / swarm_traj remaps differ between
    # simulated swarm (shared topics) and realworld deploy (per-drone suffix).
    def swarm_target(topic):
        if realworld:
            return f'{topic}_{drone_id}'
        return topic

    exploration_node = Node(
        package='exploration_manager',
        executable='exploration_node',
        name=f'exploration_node_{drone_id}',
        output='log',
        remappings=[
            ('/odom_world', P('odometry_topic')),
            ('/map_ros/pose', P('sensor_pose_topic')),
            ('/map_ros/depth', P('depth_topic')),
            ('/map_ros/cloud', P('cloud_topic')),
            ('/planning/replan', s('/planning/replan')),
            ('/planning/new', s('/planning/new')),
            ('/planning/bspline', s('/planning/bspline')),
            ('/swarm_expl/drone_state_send', swarm_target('/swarm_expl/drone_state_send') if realworld else '/swarm_expl/drone_state'),
            ('/swarm_expl/drone_state_recv', swarm_target('/swarm_expl/drone_state_recv') if realworld else '/swarm_expl/drone_state'),
            ('/swarm_expl/pair_opt_send', swarm_target('/swarm_expl/pair_opt_send') if realworld else '/swarm_expl/pair_opt'),
            ('/swarm_expl/pair_opt_recv', swarm_target('/swarm_expl/pair_opt_recv') if realworld else '/swarm_expl/pair_opt'),
            ('/swarm_expl/pair_opt_res_send', swarm_target('/swarm_expl/pair_opt_res_send') if realworld else '/swarm_expl/pair_opt_res'),
            ('/swarm_expl/pair_opt_res_recv', swarm_target('/swarm_expl/pair_opt_res_recv') if realworld else '/swarm_expl/pair_opt_res'),
            ('/swarm_expl/grid_tour_send', swarm_target('/swarm_expl/grid_tour_send') if realworld else '/swarm_expl/grid_tour'),
            ('/swarm_expl/hgrid_send', swarm_target('/swarm_expl/hgrid_send') if realworld else '/swarm_expl/hgrid'),
            ('/multi_map_manager/chunk_stamps_send', swarm_target('/multi_map_manager/chunk_stamps_send') if realworld else '/multi_map_manager/chunk_stamps'),
            ('/multi_map_manager/chunk_data_send', swarm_target('/multi_map_manager/chunk_data_send') if realworld else '/multi_map_manager/chunk_data'),
            ('/multi_map_manager/chunk_stamps_recv', swarm_target('/multi_map_manager/chunk_stamps_recv') if realworld else '/multi_map_manager/chunk_stamps'),
            ('/multi_map_manager/chunk_data_recv', swarm_target('/multi_map_manager/chunk_data_recv') if realworld else '/multi_map_manager/chunk_data'),
            ('/planning/swarm_traj_recv', swarm_target('/planning/swarm_traj_recv') if realworld else '/planning/swarm_traj'),
            ('/planning/swarm_traj_send', swarm_target('/planning/swarm_traj_send') if realworld else '/planning/swarm_traj'),
            ('/planning_vis/trajectory', s('/planning_vis/trajectory')),
            ('/planning_vis/frontier', s('/planning_vis/frontier')),
            ('/planning_vis/viewpoints', s('/planning_vis/viewpoints')),
            ('/sdf_map/occupancy_all', s('/sdf_map/occupancy_all')),
            ('/sdf_map/occupancy_local', s('/sdf_map/occupancy_local')),
            ('/sdf_map/occupancy_local_inflate', s('/sdf_map/occupancy_local_inflate')),
            ('/sdf_map/unknown', s('/sdf_map/unknown')),
            ('/sdf_map/update_range', s('/sdf_map/update_range')),
            ('/sdf_map/basecoor', basecoor_target),
        ],
        parameters=[{
            # sdf_map ---------------------------------------------------------
            'sdf_map.resolution': 0.1,
            'sdf_map.map_size_x': float(P('map_size_x')),
            'sdf_map.map_size_y': float(P('map_size_y')),
            'sdf_map.map_size_z': float(P('map_size_z')),
            'sdf_map.obstacles_inflation': 0.199,
            'sdf_map.local_bound_inflate': 0.5,
            'sdf_map.local_map_margin': 50,
            'sdf_map.ground_height': -1.0,
            'sdf_map.default_dist': 0.5,
            'sdf_map.p_hit': 0.65,
            'sdf_map.p_miss': 0.35,
            'sdf_map.p_min': 0.12,
            'sdf_map.p_max': 0.90,
            'sdf_map.p_occ': 0.80,
            'sdf_map.min_ray_length': 0.5,
            'sdf_map.max_ray_length': 4.5,
            'sdf_map.virtual_ceil_height': -10.0,
            'sdf_map.optimistic': True,
            'sdf_map.signed_dist': False,
            # Default bounds match the pillar environment (see single_drone_planner.xml).
            'sdf_map.box_min_x': -7.0,
            'sdf_map.box_min_y': -15.0,
            'sdf_map.box_min_z': 0.0,
            'sdf_map.box_max_x': 7.0,
            'sdf_map.box_max_y': 15.0,
            'sdf_map.box_max_z': 1.7,
            'sdf_map.no_drone_1': False,

            # map_ros ---------------------------------------------------------
            'map_ros.cx': float(P('cx')),
            'map_ros.cy': float(P('cy')),
            'map_ros.fx': float(P('fx')),
            'map_ros.fy': float(P('fy')),
            'map_ros.depth_filter_maxdist': 4.6,
            'map_ros.depth_filter_mindist': 0.2,
            'map_ros.depth_filter_margin': 2,
            'map_ros.k_depth_scaling_factor': 1000.0,
            'map_ros.skip_pixel': 2,
            'map_ros.esdf_slice_height': 0.5,
            'map_ros.visualization_truncate_height': 10.09,
            'map_ros.visualization_truncate_low': -2.0,
            'map_ros.show_occ_time': False,
            'map_ros.show_esdf_time': False,
            'map_ros.show_all_map': True,
            'map_ros.frame_id': 'world',

            # fsm -------------------------------------------------------------
            'fsm.thresh_replan1': 0.2,
            'fsm.thresh_replan2': 0.2,
            'fsm.thresh_replan3': 1.5,
            'fsm.replan_time': 0.200,
            'fsm.sync_interval': 0.200,
            'fsm.wait_delete_duration': 0.05,
            'fsm.gain_thresh': 10,
            'fsm.attempt_interval': 0.1,
            'fsm.pair_opt_interval': 0.5,

            # partitioning ----------------------------------------------------
            'partitioning.min_unknown': 4000,
            'partitioning.min_frontier': 100,
            'partitioning.min_free': 3000,
            'partitioning.consistent_cost': -5.0,
            'partitioning.consistent_cost2': 8.0,
            'partitioning.w_unknown': 0.0,
            'partitioning.grid_size': 5.0,
            'partitioning.use_swarm_tf': True,

            # exploration -----------------------------------------------------
            'exploration.refine_local': True,
            'exploration.refined_num': 7,
            'exploration.refined_radius': 5.0,
            'exploration.max_decay': 0.8,
            'exploration.top_view_num': 15,
            'exploration.vm': 1.0 * float(P('max_vel')),
            'exploration.am': 1.0 * float(P('max_acc')),
            'exploration.yd': 80.0 * math.pi / 180.0,
            'exploration.ydd': 90.0 * math.pi / 180.0,
            'exploration.w_dir': 1.5,
            'exploration.tsp_dir': tsp_dir,
            'exploration.mtsp_dir': mtsp_dir,
            'exploration.drone_num': int(drone_num),
            'exploration.drone_id': int(drone_id),
            'exploration.init_plan_num': 2,

            # frontier --------------------------------------------------------
            'frontier.cluster_min': 100,
            'frontier.cluster_size_xy': 2.0,
            'frontier.cluster_size_z': 10.0,
            'frontier.min_candidate_dist': 0.5,
            'frontier.min_candidate_clearance': 0.21,
            'frontier.candidate_dphi': 15.0 * math.pi / 180.0,
            'frontier.candidate_rnum': 3,
            'frontier.candidate_rmin': 1.0,
            'frontier.candidate_rmax': 1.5,
            'frontier.down_sample': 3,
            'frontier.min_visib_num': 30,
            'frontier.min_view_finish_fraction': 0.2,

            # perception_utils ------------------------------------------------
            'perception_utils.top_angle': 0.56125,
            'perception_utils.left_angle': 0.69222,
            'perception_utils.right_angle': 0.68901,
            'perception_utils.max_dist': 4.5,
            'perception_utils.vis_dist': 1.0,

            # heading_planner -------------------------------------------------
            'heading_planner.yaw_diff': 30.0 * math.pi / 180.0,
            'heading_planner.half_vert_num': 5,
            'heading_planner.lambda1': 2.0,
            'heading_planner.lambda2': 1.0,
            'heading_planner.max_yaw_rate': 10.0 * math.pi / 180.0,
            'heading_planner.w': 20000.0,
            'heading_planner.weight_type': 1.0,

            # manager ---------------------------------------------------------
            'manager.max_vel': float(P('max_vel')),
            'manager.max_acc': float(P('max_acc')),
            'manager.max_jerk': 4.0,
            'manager.dynamic_environment': 0,
            'manager.local_segment_length': 6.0,
            'manager.clearance_threshold': 0.2,
            'manager.control_points_distance': 0.5,
            'manager.use_geometric_path': True,
            'manager.use_kinodynamic_path': True,
            'manager.use_topo_path': False,
            'manager.use_optimization': True,
            'manager.use_active_perception': True,
            'manager.min_time': True,
            'manager.relax_time1': 0.3,
            'manager.relax_time2': 1.5,
            'manager.max_yawdot': 120.0 * math.pi / 180.0,

            # kinodynamic search ---------------------------------------------
            'search.max_tau': 0.8,
            'search.init_max_tau': 1.0,
            'search.max_vel': float(P('max_vel')),
            'search.vel_margin': 0.25,
            'search.max_acc': float(P('max_acc')),
            'search.w_time': 10.0,
            'search.horizon': 5.0,
            'search.lambda_heu': 10.0,
            'search.resolution_astar': 0.025,
            'search.time_resolution': 0.8,
            'search.margin': 0.2,
            'search.allocate_num': 100000,
            'search.check_num': 10,
            'search.optimistic': False,

            'astar.lambda_heu': 10000.0,
            'astar.resolution_astar': 0.3,
            'astar.allocate_num': 1000000,
            'astar.max_search_time': 0.001,

            # trajectory optimization ----------------------------------------
            'optimization.ld_smooth': 5.0,
            'optimization.ld_dist': 10.0,
            'optimization.ld_feasi': 2.0,
            'optimization.ld_start': 100.0,
            'optimization.ld_end': 0.5,
            'optimization.ld_guide': 1.5,
            'optimization.ld_waypt': 7.0,
            'optimization.ld_view': 0.0,
            'optimization.ld_time': 1.5,
            'optimization.ld_swarm': 5.0,
            'optimization.swarm_safe_dist': 1.0,
            'optimization.dist0': 0.7,
            'optimization.max_vel': float(P('max_vel')),
            'optimization.max_acc': float(P('max_acc')),
            'optimization.algorithm1': 15,
            'optimization.algorithm2': 11,
            'optimization.max_iteration_num1': 2,
            'optimization.max_iteration_num2': 2000,
            'optimization.max_iteration_num3': 200,
            'optimization.max_iteration_num4': 200,
            'optimization.max_iteration_time1': 0.0001,
            'optimization.max_iteration_time2': 0.005,
            'optimization.max_iteration_time3': 0.003,
            'optimization.max_iteration_time4': 0.003,

            # bspline ---------------------------------------------------------
            'bspline.limit_vel': float(P('max_vel')),
            'bspline.limit_acc': float(P('max_acc')),
            'bspline.limit_ratio': 1.1,
        }] + ([{
            # realworld overrides — from single_drone_planner_realworld.xml.
            'sdf_map.p_hit': 0.68,
            'sdf_map.p_miss': 0.4,
            'sdf_map.p_max': 0.99,
            'sdf_map.p_occ': 0.70,
            'sdf_map.max_ray_length': 3.5,
            'sdf_map.box_min_x': -6.5,
            'sdf_map.box_min_y': -2.2,
            'sdf_map.box_min_z': 0.1,
            'sdf_map.box_max_x': 2.5,
            'sdf_map.box_max_y': 3.2,
            'sdf_map.box_max_z': 1.4,
            'sdf_map.no_drone_1': single_expo,
            'map_ros.depth_filter_maxdist': 3.5,
            'map_ros.esdf_slice_height': 0.3,
            'map_ros.visualization_truncate_height': 1.5,
            'fsm.thresh_replan2': 0.5,
            'fsm.attempt_interval': 0.4,
            'fsm.pair_opt_interval': 2.0,
            'partitioning.min_unknown': 500,
            'partitioning.grid_size': 4.0,
            'exploration.yd': 45.0 * math.pi / 180.0,
            'frontier.min_visib_num': 10,
            'search.max_tau': 1.0,
            'search.horizon': 6.0,
            'optimization.ld_time': 1.0,
            'optimization.swarm_safe_dist': 1.5,
            'optimization.dist0': 0.8,
            'optimization.max_iteration_time2': 0.015,
        }] if realworld else []) + ([{
            # Quadruped overlay (robot_type:=quadruped). Pins planning to a
            # thin z-slab around stand_height, tightens velocity/yaw caps,
            # and shortens viewpoint scan radius to quadruped-realistic
            # values. Applied on top of default/realworld params.
            'sdf_map.box_min_z': stand_height - 0.05,
            'sdf_map.box_max_z': stand_height + 0.05,
            'sdf_map.ground_height': 0.0,
            'sdf_map.virtual_ceil_height': stand_height + 0.20,
            'manager.max_vel': 1.0,
            'manager.max_acc': 0.5,
            'manager.max_yawdot': 1.0,  # rad/s — was 120 deg/s = 2.09 rad/s
            'optimization.max_vel': 1.0,
            'optimization.max_acc': 0.5,
            'optimization.max_vel_z': 0.1,   # quadruped: tight z cap
            'optimization.max_acc_z': 0.1,
            'search.max_vel': 1.0,
            'search.max_acc': 0.5,
            'bspline.limit_vel': 1.0,
            'bspline.limit_acc': 0.5,
            'exploration.vm': 1.0,
            'exploration.am': 0.5,
            'exploration.yd': 45.0 * math.pi / 180.0,
            'exploration.ydd': 57.0 * math.pi / 180.0,
            'frontier.candidate_rmin': 0.8,
            'frontier.candidate_rmax': 1.2,
            'frontier.ground_clearance': max(0.0, stand_height - 0.05),
            'frontier.viewpoint_z_pin': stand_height,
        }] if quadruped else []),
    )

    mtsp_node = Node(
        package='lkh_mtsp_solver',
        executable='mtsp_node',
        name=f'tsp_solver_{drone_id}',
        output='log',
        parameters=[{
            'exploration.drone_id': int(drone_id),
            'exploration.mtsp_dir': mtsp_dir,
            'exploration.problem_id': 1,
        }],
    )

    acvrp_node = Node(
        package='lkh_mtsp_solver',
        executable='mtsp_node',
        name=f'acvrp_solver_{drone_id}',
        output='log',
        parameters=[{
            'exploration.drone_id': int(drone_id),
            'exploration.mtsp_dir': mtsp_dir,
            'exploration.problem_id': 2,
        }],
    )

    return [exploration_node, mtsp_node, acvrp_node]


def generate_launch_description():
    return LaunchDescription(_args() + [OpaqueFunction(function=_build)])
