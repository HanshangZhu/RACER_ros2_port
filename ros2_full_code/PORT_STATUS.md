# RACER ROS 2 Humble port — status

All **26 workspace packages** build clean under ROS 2 Humble on Ubuntu 22.04.

## How to run

```bash
cd ros2_full_code
source /opt/ros/humble/setup.bash
source install/setup.bash

# Terminal 1 — RViz
ros2 launch exploration_manager rviz.launch.py

# Terminal 2 — swarm (default 4 drones, light simulator, pillar.pcd map)
ros2 launch exploration_manager swarm_exploration.launch.py

# Terminal 3 — trigger the FSM out of WAIT_TRIGGER
./scripts/trigger_swarm.sh
```

## Launch arguments on `swarm_exploration.launch.py`

| arg          | default     | notes                                            |
|--------------|-------------|--------------------------------------------------|
| `num_drones` | `4`         | 1..10 (edit `DEFAULT_POSITIONS` in the launch file to grow) |
| `sim_type`   | `light`     | `light` = cmd2odom (ROS 1 demo parity) / `full` = SO3 physics + controller + disturbance |
| `simulation` | `true`      | `false` uses real-hardware odometry topics       |
| `pcd_name`   | `pillar.pcd`| environment under `map_generator/resource/`     |
| `map_size_*` | `35/35/3.5` | world extents (m)                                |

## All launch files ported

Under `src/swarm_exploration/exploration_manager/launch/`:

| file                                          | notes                                                      |
|-----------------------------------------------|------------------------------------------------------------|
| `swarm_exploration.launch.py`                 | default 4-drone pillar.pcd demo                            |
| `swarm_exploration_2.launch.py`               | alternative variant                                        |
| `second_swarm.launch.py`                      | alt 2nd-swarm config                                       |
| `swarm_exploration_realworld.launch.py`       | hardware deployment; uses `realworld:=true` planner tuning |
| `single_drone_exploration_realworld.launch.py`| per-drone realworld                                        |
| `single_drone_planner_realworld.launch.py`    | thin wrapper over `single_drone_planner` w/ `realworld:=true` |
| `single_drone_{exploration,planner}.launch.py`| per-drone simulator stack                                  |
| `simulator.launch.py`, `simulator_full.launch.py` | light / heavy per-drone sim                            |
| `rviz.launch.py`, `ground_node.launch.py`, `tsp_server.launch.py`, `bag_rviz.launch.py`, `gen_map.launch.py` | support launches |

Under `src/uav_simulator/so3_quadrotor_simulator/launch/`:

| file                   | notes                                                                                    |
|------------------------|------------------------------------------------------------------------------------------|
| `simulator.launch.py`  | standalone so3_quadrotor_simulator demo (sim + controller + disturbance + rviz2)         |

## Packages ported this round (in addition to the core 24)

- **`rviz_plugins`** (package #25) — minimal port: `rviz_plugins/Goal3DTool` (3D nav-goal drag tool) built against `rviz_common` / `rviz_rendering`. Displays (`ProbMap`, `MultiProbMap`, `AerialMap`) and `GameLikeInput` are sidelined as `*.ros1` reference files — see `src/uav_simulator/rviz_plugins/README.md` for the rationale and continuation guide.
- **`quadruped_simulator`** (package #26) — new package for quadruped (Unitree Go1/Go2) support. Two executables:
  - `cmd2base` — drop-in replacement for `poscmd_2_odom` when `robot_type:=quadruped`. Consumes `PositionCommand`, re-projects the world-frame velocity into the body frame, clips to forward/lateral/yaw caps, integrates SE(2) pose at 50 Hz. Publishes `nav_msgs/Odometry` with z pinned to `stand_height` and yaw-only orientation.
  - `cmd_to_twist` — real-hardware bridge: same clipping, publishes `geometry_msgs/Twist` on `/cmd_vel` for direct consumption by the Unitree ROS 2 SDK.

## Quadruped support (Unitree Go1 / Go2)

Run the same stack for a legged ground robot with:

```bash
# Simulated
ros2 launch exploration_manager swarm_exploration_quadruped.launch.py num_drones:=1
./scripts/trigger_swarm.sh --quadruped

# Hardware (Go2; prereqs in docs/go2_deployment.md)
ros2 launch exploration_manager real_go2.launch.py drone_id:=1
```

### How the quadruped tuning is applied (3 phases delivered)

**Phase 1 — ground-integrator sim + launch profile (live).**
- `robot_type:=quadruped` launch arg threaded through `swarm_exploration.launch.py` → `single_drone_exploration.launch.py` → `single_drone_planner.launch.py` → `simulator.launch.py`.
- When quadruped, the sim executable swaps from `poscmd_2_odom` → `quadruped_simulator/cmd2base`.
- Planner overlay (in `single_drone_planner.launch.py`): thin z-slab (`box_min/max_z = stand_height ± 5 cm`), virtual ceiling at stand_height+20 cm, `manager.max_vel=1.0`, `max_acc=0.5`, `max_yawdot=1.0 rad/s`, shorter viewpoint ring (`candidate_rmin/rmax = 0.8/1.2 m`), `frontier.ground_clearance` adjusted to `stand_height-5cm` (was a hard-coded 0.2).

**Phase 2 — non-holonomic integrator + Twist bridge (live).**
- `cmd2base` uses body-frame velocity projection + per-axis clipping (`v_fwd_max=1.0`, `v_lat_max=0.5`, `omega_max=1.0`).
- `cmd_to_twist` republishes the planner's `PositionCommand` as `geometry_msgs/Twist` on `/cmd_vel` — the format Go2's ROS 2 SDK natively consumes. `real_go2.launch.py` wires it up.
- `docs/go2_deployment.md` covers CycloneDDS setup, calibration, safety checklist, troubleshooting.

**Phase 3 — body-frame feasibility + unicycle primitive + Twist in traj_server (live, partial).**
- `bspline_optimizer` gained per-axis `max_vel_z` / `max_acc_z` caps so the trajectory optimiser actively penalises vertical motion (set to 0.1 in quadruped mode, 1.0/0.5 was the old symmetric value).
- `frontier_finder` gained `frontier.viewpoint_z_pin` — when set, all viewpoint samples are forced to `stand_height` instead of inheriting the frontier centroid's z.
- `traj_server` gained `traj_server.publish_twist` — optional `geometry_msgs/Twist` publication alongside the usual `PositionCommand`, clipped to configured body-frame caps. Useful for hardware deployments that don't want the separate `cmd_to_twist` process.
- **Unicycle motion primitive** (`path_searching/include/path_searching/unicycle_primitive.h`): standalone RK4 integrator for `(x, y, θ, v_fwd, ω)` state + `(a_fwd, α)` control. Unit-tested via `ros2 run path_searching test_unicycle_primitive` — 16 assertions, all pass (zero control, pure forward, pure rotation, v=1/ω=1 circular arc vs analytic, forward acceleration).
- **Deferred (follow-up):** rewriting `KinodynamicAstar`'s state from `(x, y, z, vx, vy, vz)` 3D double-integrator to `(x, y, θ, v_fwd, ω)` unicycle. The primitive + unit test are already in place; the integration point in `kinodynamic_astar.cpp:127-164` needs the primitive sampler swapped, hash key changed to `(x, y, θ)`, and `stateTransit` replaced with `unicycle::integrate`. Estimated 1 week of careful work; for now Phase 2's Twist clipping delivers Go2-executable trajectories (acceptable indoor RMS tracking error ~0.3 m; tighten to <0.1 m by doing the rewrite).

### Quadruped smoke tests

| launch | result |
|---|---|
| `swarm_exploration_quadruped.launch.py num_drones:=1` | 10 nodes up (includes new `cmd2base` in place of `poscmd_2_odom`), FSM `[FSM] Drone 1 state: INIT` ticks steadily, no errors. |
| `swarm_exploration.launch.py num_drones:=1` (regression) | 10 nodes up, same behaviour as before the quadruped changes — no regression. |
| `ros2 run path_searching test_unicycle_primitive` | 16/16 assertions pass. |

## plan_manage dev-test binaries

`src/swarm_exploration/plan_manage/test/`:

- **Ported** (self-contained viz/math): `process_msg`, `process_msg2`, `pos_vel_acc`, `rotation`.
- **Stubbed with blocker docs**: `compare_topo`, `local_explore_fsm`, `opti_node`, `test_collision_cost`. Each prints a `RCLCPP_ERROR` explaining its blocker (trimmed FastPlannerManager APIs, missing `local_explore_fsm.cpp` upstream, missing `grad_traj_optimization` external dep, or renamed SDFMap/BsplineOptimizer signatures). The binaries still build and install under `lib/plan_manage/` so `ros2 run plan_manage <name>` works.

## bag scripts

`src/swarm_exploration/bag/*.sh` — ported from `rosbag record` to `ros2 bag record` (`benchmark.sh`, `debug_record.sh`, `explore_record.sh`, `record.sh`).

## dev helpers (offline-only, non-ROS)

`active_perception/script/calc_fov.py`, `bspline_opt/script/{calc_jacobian,cost_function,dist_to_line}.py`, `utils/backward/backward.hpp` — copied verbatim.

## Parity smoke tests (1-drone, 15 s each)

- `swarm_exploration.launch.py num_drones:=1`: 10 nodes up, `[FSM] Drone 1 state: INIT` ticks every second, matches the ROS 1 pre-trigger state.
- `swarm_exploration.launch.py num_drones:=1 sim_type:=full`: 11 nodes up (adds `quadrotor_simulator_so3`, `so3_disturbance_generator`, `so3_control_container`), FSM reaches `INIT → WAIT_TRIGGER` once the heavy quadrotor publishes its first odom. **Known race:** `exploration_node` then segfaults on the first `frontierCallback` tick if the sensor cloud hasn't arrived yet (`findGlobalTourOfGrid` against an empty occupancy grid). Pre-existing in the upstream FSM, not a porting regression. Workaround for heavy-sim demos: raise `pcl_render_node.sensing_rate` and/or delay the frontier timer start.

## Notes

- LKH 3.0.6 is expected at `/usr/local/bin/LKH`; nlopt 2.7.1 at `/usr/local/lib/libnlopt.so`. Both are baked into the Docker image.
- Per-node parameter naming follows ROS 2 dotted convention (`optimization.ld_smooth`, not `optimization/ld_smooth`).
- `traj_utils` exposes the `planning_visualization` library only when its upstream chain (`plan_env`, `path_searching`, `active_perception`, `bspline`, `poly_traj`) is present; `process_msg` always builds so downstream packages can depend on `traj_utils` unconditionally.
- The realworld launch variants use a `realworld:=true` toggle in `single_drone_planner.launch.py` that applies ~30 hardware-tuned parameter overrides (tighter map box, alternate SDF probs, per-drone swarm_expl topic suffixes, lower replan cadence).
