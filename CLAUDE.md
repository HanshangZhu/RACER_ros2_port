# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Platform constraint

RACER is a **ROS 1 (Melodic/Noetic) only** project — the source makes no effort to compile under ROS 2. The host here is Ubuntu 22.04 + ROS Humble, so all work happens inside the ROS Noetic Docker image in [docker/](docker/). Do not attempt a native build on the host; `catkin_make` and `ros-noetic-*` are not installed there.

A native ROS 2 Humble port is a planned follow-up task, tracked separately.

## Docker workflow

Container name is `racer`, image is `racer:noetic`. The host repo is bind-mounted at `/catkin_ws/src/RACER`; catkin `build/` and `devel/` persist under `docker/.cache/` on the host.

```bash
./docker/build.sh                       # build/refresh image (only when Dockerfile changes)
./docker/run.sh                         # start interactive container with X11 passthrough
./docker/exec.sh                        # open extra shell in already-running container
docker stop racer && docker rm racer    # tear down
```

Typical dev loop, once a container is running:

```bash
# inside container — ROS + devel are auto-sourced by .bashrc
cd /catkin_ws && catkin_make -j$(nproc)                    # incremental rebuild after code changes
roslaunch exploration_manager rviz.launch                  # terminal 1
roslaunch exploration_manager swarm_exploration.launch     # terminal 2 — then click "2D Nav Goal" in Rviz to trigger
```

Build artifacts in `docker/.cache/{build,devel}` are root-owned (container runs as root). If you need to wipe them from the host: `sudo rm -rf docker/.cache`.

## External solver dependencies

Both are baked into the image, not vendored in the repo — changes to the Dockerfile are needed to pin different versions:

- **LKH 3.0.6** at `/usr/local/bin/LKH`. Invoked via ROS services by `utils/lkh_tsp_solver` (single-salesman) and `utils/lkh_mtsp_solver` (multi-salesman, ACVRP). `exploration_manager` calls `/solve_tsp_*` and `/solve_acvrp_*` from [fast_exploration_manager.cpp](swarm_exploration/exploration_manager/src/fast_exploration_manager.cpp).
- **nlopt v2.7.1** installed to `/usr/local/lib/libnlopt.so`. Used by `bspline_opt/bspline_optimizer` for trajectory refinement. CMake in `bspline_opt` hard-codes `/usr/local/lib/libnlopt.so` — rebuilding nlopt elsewhere requires editing that path.

## Architecture

Two top-level groups of packages:

**`swarm_exploration/`** — the planner and coordination stack
- `exploration_manager` — per-drone FSM ([fast_exploration_fsm.cpp](swarm_exploration/exploration_manager/src/fast_exploration_fsm.cpp)); builds `exploration_node` (per drone) and `ground_node` (optional centralized map/grid aggregator).
- `active_perception` — frontier clustering, viewpoint generation, visibility/gain computation.
- `plan_env` — SDF/ESDF occupancy grid, raycast, multi-agent map fusion.
- `plan_manage` — planner orchestration; builds `traj_server` (consumes B-splines, publishes position commands).
- `bspline`, `bspline_opt` — B-spline parameterization + nlopt-backed trajectory refinement.
- `path_searching`, `poly_traj` — kinodynamic A*/RRT seeds and polynomial segments.
- `utils` — LKH wrappers (`lkh_tsp_solver`, `lkh_mtsp_solver`) exposing TSP/ACVRP as services.
- `traj_utils`, `bag` — message types and bag-replay helpers.

**`uav_simulator/`** — the simulation stack (only used when `simulation:=true`)
- `map_generator` — loads `.pcd` environments from [resource/](uav_simulator/map_generator/resource/) and publishes as a point cloud.
- `so3_quadrotor_simulator` — rigid-body quadrotor dynamics.
- `so3_control` — SO(3) geometric controller; closes the loop against the dynamics.
- `local_sensing` — depth-camera simulator (`pcl_render_node`). **CUDA is disabled by default** in its [CMakeLists.txt](uav_simulator/local_sensing/CMakeLists.txt) (`set(ENABLE_CUDA false)`); flip it and rebuild the image with CUDA deps if GPU-accelerated depth rendering is needed.
- `poscmd_2_odom`, `so3_disturbance_generator`, `Utils` — command-to-odom loopback, disturbance injection, message/viz helpers.

## Launch file hierarchy

Under [swarm_exploration/exploration_manager/launch/](swarm_exploration/exploration_manager/launch/):

- `swarm_exploration.launch` — top-level demo; by default spawns **4 drones** and the pillar environment. Configuration happens by **editing the launch file**, not through command-line args (drone instances are enumerated explicitly inside the file).
- `simulator.xml` — per-drone simulation stack (dynamics + controller + depth sensing + odom).
- `single_drone_exploration.xml` — per-drone exploration stack; includes `single_drone_planner.xml` + `simulator.xml`.
- `single_drone_planner.xml` — per-drone planning parameters; this is where map bounding box (`box_min_*`, `box_max_*`) and frontier/exploration thresholds live.
- `tsp_server.launch` — standalone LKH service nodes (included by `swarm_exploration.launch`).
- `ground_node.launch` — optional ground aggregator.
- `*_realworld.*` — hardware-deployment variants (no simulator).

When changing maps or bounds, the two files to edit are `swarm_exploration.launch` (pcd path) and `single_drone_planner.xml` (box bounds) — they are not kept in sync automatically.

## Known benign warnings

During startup, before the user hits "2D Nav Goal" in Rviz, the log floods with `[ERROR] Path N inconsistent` and `[ERROR] Larger cost after reallocation`. These come from the pair-optimization allocation loop in `exploration_manager` running against empty drone states. They are **not fatal** and resolve as soon as a trigger is received. Don't investigate them unless they persist after the swarm is triggered.

The compile-time warnings (signed/unsigned comparisons, unused variables) are upstream and not worth fixing unless touching the surrounding code.

## Creating new environments

`.pcd` files in [uav_simulator/map_generator/resource/](uav_simulator/map_generator/resource/) are the environment catalog. Adding a new one requires: (1) drop the `.pcd`, (2) point `map_pub` in `swarm_exploration.launch` at it, (3) adjust `box_min_*`/`box_max_*` in `single_drone_planner.xml` to the new extents. The README links to FUEL's `click_map` tool for authoring `.pcd` environments.
