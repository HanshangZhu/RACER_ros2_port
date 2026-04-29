# RACER → ROS 2 Native Port Plan

## Context

The upstream codebase is ROS 1 (Melodic/Noetic) and currently runs on this host only via the Docker-Noetic image in [docker/](docker/). The planned next step is a **full native ROS 2 port** that runs on the host's ROS 2 distribution with no ROS 1 bridge, no Noetic Docker. Scope is **all 26 packages** — we are extending beyond the demo, so dead-in-demo packages (`multi_map_server`, `waypoint_generator`, `so3_disturbance_generator`, `rviz_plugins`, etc.) must be ported too. Ported sources live in [ros2_full_code/](ros2_full_code/) as a **standalone colcon workspace**; the ROS 1 tree ([swarm_exploration/](swarm_exploration/), [uav_simulator/](uav_simulator/)) stays untouched for side-by-side reference.

## Target

- **ROS 2 distribution: Humble Hawksbill.** Matches the host (Ubuntu 22.04) and is the current LTS with longest support runway. No ROS 1 bridge — this is a clean rewrite against `rclcpp` / `ament_cmake` / `rosidl`.
- **Workspace layout:**
  ```
  ros2_full_code/
    src/
      swarm_exploration/          # mirrors upstream tree
        active_perception/
        bspline/
        bspline_opt/
        exploration_manager/
        path_searching/
        plan_env/
        plan_manage/
        poly_traj/
        traj_utils/
        utils/
          lkh_mtsp_solver/
          lkh_tsp_solver/
      uav_simulator/
        local_sensing/
        map_generator/
        poscmd_2_odom/
        so3_control/
        so3_disturbance_generator/
        so3_quadrotor_simulator/
        Utils/
          cmake_utils/
          multi_map_server/
          odom_visualization/
          pose_utils/
          quadrotor_msgs/
          rviz_plugins/
          uav_utils/
          waypoint_generator/
    README_ROS2.md                # native build/run instructions
  ```
- **Build tool:** `colcon build --symlink-install`.
- **Messaging IDL:** each `.msg` / `.srv` stays in its current package (ROS 2 supports in-package IDL with `rosidl_default_generators`).

## Scope decisions, up front

- **Dedupe `quadrotor_msgs`.** There are two copies on disk: `uav_simulator/Utils/quadrotor_msgs/` (canonical) and `uav_simulator/Utils/multi_map_server/quadrotor_msgs/` (nested leftover, identical content). Port only the canonical one; delete the nested copy in the ROS 2 tree.
- **`inf_uwb_ros` is external.** It's referenced by `swarm_exploration_realworld.launch` and friends but is not in this repo. Leave it out of `ros2_full_code/` and declare the realworld launches as "requires external `inf_uwb_ros` ROS 2 port, not vendored here." Port the sim launches first; realworld launches are ported but will fail to launch until the UWB package is available.
- **`dynamic_reconfigure` goes away.** Used in `so3_disturbance_generator` (main consumer) and `local_sensing/pcl_render_node`. Replace with ROS 2 parameters + `add_on_set_parameters_callback`. The `DisturbanceUIConfig.h` generated header disappears — write a small parameter struct instead.
- **Single nodelet (`so3_control/so3_control_nodelet.cpp`).** Becomes a `rclcpp_components::Component` registered via `RCLCPP_COMPONENTS_REGISTER_NODE`. No standalone binary; launch as a `ComposableNode` in a `ComponentContainer`.
- **`tf` (tf1) → `tf2_ros`.** 10 files use old `tf::`; all migrate to `tf2_ros::Buffer` + `TransformListener` and `tf2::` math types.
- **`boost::shared_ptr` / `boost::bind` / `boost::function` → `std::`.** 53 files touch boost in ways that overlap with C++17 stdlib. Mass-replace; keep boost only where it's genuinely needed (e.g., `boost::filesystem` is no longer — prefer `std::filesystem`).

## Port order

Port bottom-up so each wave compiles against already-ported dependencies. Each wave must build green before starting the next.

**Wave 0 — Workspace scaffold.**
  - Create `ros2_full_code/src/` and empty mirror directories.
  - Write `ros2_full_code/README_ROS2.md` + a `colcon defaults.yaml` if we want selective builds.
  - Add a `docker-ros2/` image (ROS 2 Humble desktop + LKH 3.0.6 + nlopt 2.7.1 + armadillo) analogous to [docker/Dockerfile](docker/Dockerfile). Host has Humble already, but a container keeps the build reproducible.

**Wave 1 — Interface packages (pure IDL, no C++ ports needed).**
  - [quadrotor_msgs](uav_simulator/Utils/quadrotor_msgs/) — 12 msgs (AuxCommand, Corrections, Gains, LQRTrajectory, Odometry, PolynomialTrajectory, PositionCommand, PPROutputData, SO3Command, StatusData, Serial, TRPYCommand).
  - [bspline](swarm_exploration/bspline/) — Bspline.msg (plus math headers, see Wave 2).
  - [plan_env](swarm_exploration/plan_env/) interfaces — ChunkData, ChunkStamps, IdxList (rest of package is Wave 4).
  - [exploration_manager](swarm_exploration/exploration_manager/) interfaces — DroneState, PairOpt, PairOptResponse, HGrid, GridTour (rest is Wave 6).
  - [multi_map_server](uav_simulator/Utils/multi_map_server/) interfaces — MultiOccupancyGrid, MultiSparseMap3D, SparseMap3D, VerticalOccupancyGridList.
  - [lkh_tsp_solver](swarm_exploration/utils/lkh_tsp_solver/) — SolveTSP.srv.
  - [lkh_mtsp_solver](swarm_exploration/utils/lkh_mtsp_solver/) — SolveMTSP.srv.
  - For each: port `package.xml` to format 3, `CMakeLists.txt` to `ament_cmake` + `rosidl_default_generators`, add `rosidl_interfaces` target.

**Wave 2 — Header-only / non-ROS math libs.**
  - [cmake_utils](uav_simulator/Utils/cmake_utils/) — shared CMake helpers; port to `ament_cmake` exports.
  - [pose_utils](uav_simulator/Utils/pose_utils/) — math only; minimal changes.
  - [uav_utils](uav_simulator/Utils/uav_utils/) — math/geometry helpers; audit for any `ros/ros.h` includes.
  - [poly_traj](swarm_exploration/poly_traj/) — polynomial trajectory math (mostly `ros::Time` — needs `rclcpp::Time` or keep as `double` seconds internally).
  - [bspline](swarm_exploration/bspline/) C++ — B-spline parameterization, mostly math.

**Wave 3 — Low-level simulator nodes.**
  - [map_generator](uav_simulator/map_generator/) — loads `.pcd`, publishes PointCloud2. Simple ROS 2 conversion (create_publisher + `on_timer`).
  - [so3_quadrotor_simulator](uav_simulator/so3_quadrotor_simulator/) — dynamics node.
  - [so3_control](uav_simulator/so3_control/) — controller. **This is the nodelet surgery site** (`so3_control_nodelet.cpp` → `rclcpp_components`).
  - [poscmd_2_odom](uav_simulator/poscmd_2_odom/) — tiny.
  - [so3_disturbance_generator](uav_simulator/so3_disturbance_generator/) — drop `dynamic_reconfigure`, expose the same knobs as parameters.
  - [odom_visualization](uav_simulator/Utils/odom_visualization/) — heavy pub/sub file (11 publishers). Straight translation.
  - [waypoint_generator](uav_simulator/Utils/waypoint_generator/) — keyboard/rviz waypoint injector.
  - [rviz_plugins](uav_simulator/Utils/rviz_plugins/) — **RViz 2 plugin rewrite. Required, not deferrable.** ROS 1 `rviz` plugins do not load in RViz 2 — the base classes, plugin manifest, and Qt setup are different. Each plugin must be rewritten against `rviz_common::Display` / `rviz_rendering` / `rviz_default_plugins`, with an updated `plugins_description.xml` registered via `pluginlib::ClassList` pointing at `rviz_common::Display`. `Ogre` resource paths also differ — audit any material / mesh loading.

**Wave 4 — Planning foundations.**
  - [plan_env](swarm_exploration/plan_env/) — SDF/ESDF maps. `message_filters::Subscriber` + `TimeSynchronizer` needs ROS 2 signature (node ptr as first arg). `map_ros.cpp` is the main surgery site.
  - [path_searching](swarm_exploration/path_searching/) — A*/kino search.
  - [active_perception](swarm_exploration/active_perception/) — frontier clustering, viewpoints.
  - [bspline_opt](swarm_exploration/bspline_opt/) — nlopt-backed trajectory refiner. **Nlopt link stays the same**; the only change is replacing `ros::Time::now()` calls in the cost function timing.

**Wave 5 — Sensing + perception bridges.**
  - [local_sensing](uav_simulator/local_sensing/) — depth-cam simulator.
    - Keep `ENABLE_CUDA` off by default as upstream does; it compiles without CUDA.
    - `pcl_render_node.cpp` uses `dynamic_reconfigure` → parameters.
    - PCL/pcl_conversions are available in ROS 2; headers are the same.
  - [multi_map_server](uav_simulator/Utils/multi_map_server/) — optional centralized map server; port the C++ nodes now that its interfaces exist.

**Wave 6 — Planner orchestration + FSMs.**
  - [traj_utils](swarm_exploration/traj_utils/) — visualization helpers.
  - [plan_manage](swarm_exploration/plan_manage/) — planner_manager + traj_server. [traj_server.cpp](swarm_exploration/plan_manage/src/traj_server.cpp) has 26 time/timer refs and 6 pub/sub — dense but mechanical.
  - [utils/lkh_tsp_solver](swarm_exploration/utils/lkh_tsp_solver/) + [utils/lkh_mtsp_solver](swarm_exploration/utils/lkh_mtsp_solver/) C++ nodes — they wrap LKH via `fork/exec`; that code is unchanged. The surgery is the `advertiseService` → `create_service` + callback signature (request/response pointers).

**Wave 7 — Top-level swarm brain.**
  - [exploration_manager](swarm_exploration/exploration_manager/) — `exploration_node`, `ground_node`. This is the densest file in the repo:
    - [fast_exploration_fsm.cpp](swarm_exploration/exploration_manager/src/fast_exploration_fsm.cpp) — 35 time/timer refs.
    - [fast_exploration_manager.cpp](swarm_exploration/exploration_manager/src/fast_exploration_manager.cpp) — 48 time/timer refs + the 2 LKH service clients.
  - Port last — everything it depends on is already green.

**Wave 8 — Launch & integration.**
  - Rewrite all 27 `.launch` XML files as `.launch.py` under each package's `launch/` dir.
  - Collapse per-drone repetition using Python loops (currently `swarm_exploration.launch` inlines each drone — ROS 2 launch lets us generate them with `GroupAction` + `PushRosNamespace` inside a `for i in range(drone_num)`).
  - Port `swarm.rviz` → RViz 2 config; RViz 2 reads similar YAML but some display types were renamed. Open in RViz 2 and re-save.
  - Port `tsp_server.launch` → Python launch that spawns N TSP/MTSP services matching `drone_num`.

## Mechanical transformation reference

Keep this table on-screen while porting. It covers ~95% of the edits by volume.

| ROS 1                                                                 | ROS 2 replacement                                                              |
|-----------------------------------------------------------------------|--------------------------------------------------------------------------------|
| `ros::NodeHandle nh("~");`                                            | class owns `rclcpp::Node::SharedPtr node_;` constructed in `main` or by launcher |
| `nh.advertise<T>("topic", 10)`                                        | `node_->create_publisher<T>("topic", 10)`                                      |
| `nh.subscribe("topic", 10, &Class::cb, this)`                         | `node_->create_subscription<T>("topic", 10, std::bind(&Class::cb, this, _1))`  |
| `nh.createTimer(ros::Duration(0.1), &Class::cb, this)`                | `node_->create_wall_timer(100ms, std::bind(&Class::cb, this))`                 |
| `ros::Time::now()`                                                    | `node_->get_clock()->now()`                                                    |
| `ros::Duration(1.0).sleep()`                                          | `rclcpp::sleep_for(1s)`                                                        |
| `ros::spin()`                                                         | `rclcpp::spin(node_)`                                                          |
| `ros::AsyncSpinner`                                                   | `rclcpp::executors::MultiThreadedExecutor`                                     |
| `nh.param("k", x, default)`                                           | `node_->declare_parameter("k", default); node_->get_parameter("k", x);`        |
| `ros::ServiceClient c = nh.serviceClient<S>("n"); c.call(req, res);`  | `auto c = node_->create_client<S>("n"); auto f = c->async_send_request(req);`  |
| `nh.advertiseService("n", &Class::cb, this)`                          | `node_->create_service<S>("n", std::bind(&Class::cb, this, _1, _2))` — callback signature is `(request, response)` shared_ptrs |
| `#include <nodelet/nodelet.h>` + `PLUGINLIB_EXPORT_CLASS`             | `#include <rclcpp_components/register_node_macro.hpp>` + `RCLCPP_COMPONENTS_REGISTER_NODE(Cls)` |
| `dynamic_reconfigure::Server<C>`                                      | `node_->add_on_set_parameters_callback(cb)`                                    |
| `tf::TransformListener`                                               | `tf2_ros::Buffer` + `tf2_ros::TransformListener`                               |
| `tf::Vector3`, `tf::Quaternion`                                       | `tf2::Vector3`, `tf2::Quaternion`                                              |
| `message_filters::Subscriber<T>(nh, "t", 10)`                         | `message_filters::Subscriber<T>(node_, "t", rmw_qos_profile_default)`          |
| `boost::shared_ptr<T>`                                                | `std::shared_ptr<T>`                                                           |
| `boost::bind(&f, this, _1)`                                           | `std::bind(&f, this, std::placeholders::_1)` or lambda                         |
| `boost::function<void(...)>`                                          | `std::function<void(...)>`                                                     |
| `ROS_INFO`/`ROS_WARN`/`ROS_ERROR`                                     | `RCLCPP_INFO(node_->get_logger(), ...)` etc.                                   |

### `package.xml` (format 3 skeleton)
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>pkg_name</name>
  <version>0.0.1</version>
  <description>...</description>
  <maintainer email="...">...</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <!-- for IDL packages, instead: -->
  <!-- <buildtool_depend>rosidl_default_generators</buildtool_depend> -->
  <!-- <exec_depend>rosidl_default_runtime</exec_depend> -->
  <!-- <member_of_group>rosidl_interface_packages</member_of_group> -->
  <export><build_type>ament_cmake</build_type></export>
</package>
```

### `CMakeLists.txt` (ament skeleton)
```cmake
cmake_minimum_required(VERSION 3.14)
project(pkg_name)
if(NOT CMAKE_CXX_STANDARD) set(CMAKE_CXX_STANDARD 17) endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# ... other deps

add_executable(node_name src/main.cpp src/lib.cpp)
ament_target_dependencies(node_name rclcpp std_msgs ...)
target_link_libraries(node_name ${NLOPT_LIBRARIES})   # keep nlopt unchanged

install(TARGETS node_name DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
```

## Known surgery points

These are the handful of files where the port is non-mechanical — schedule extra time:

1. **[uav_simulator/Utils/rviz_plugins/](uav_simulator/Utils/rviz_plugins/)** — RViz plugin API changed fundamentally between ROS 1 `rviz` and ROS 2 `rviz_common`. Expect to rewrite each plugin's `onInitialize` / `onEnable` and re-register via `pluginlib` against `rviz_common::Display`. This is its own mini-project.
2. **[uav_simulator/so3_control/src/so3_control_nodelet.cpp](uav_simulator/so3_control/src/so3_control_nodelet.cpp)** — the only nodelet. Convert to `rclcpp_components` component, register with `RCLCPP_COMPONENTS_REGISTER_NODE`, update launch to use `ComposableNodeContainer`.
3. **[uav_simulator/so3_disturbance_generator/](uav_simulator/so3_disturbance_generator/)** — `dynamic_reconfigure` consumer. Replace with parameter callback; delete generated `DisturbanceUIConfig.h` machinery.
4. **[uav_simulator/local_sensing/src/pcl_render_node.cpp](uav_simulator/local_sensing/src/pcl_render_node.cpp)** — dynamic_reconfigure + PCL + optional CUDA. Port with CUDA still gated off; verify the CUDA path separately only if needed.
5. **[swarm_exploration/plan_env/src/map_ros.cpp](swarm_exploration/plan_env/src/map_ros.cpp)** — `message_filters` time synchronizer. Exact-time vs approximate-time sync policies carry over but API takes node ptr.
6. **[swarm_exploration/exploration_manager/src/fast_exploration_manager.cpp](swarm_exploration/exploration_manager/src/fast_exploration_manager.cpp)** — the 2 service clients for LKH TSP/MTSP. Convert to `async_send_request` + future; the FSM will need a spin pattern that doesn't deadlock (use a reentrant callback group or `executor.spin_until_future_complete`).
7. **FSM nested callbacks** throughout `fast_exploration_fsm.cpp`. Default ROS 2 executor is single-threaded — if FSM triggers timer callbacks from within a subscription callback, use a `MultiThreadedExecutor` with mutually exclusive callback groups to preserve ROS 1-like semantics.

## Launch file strategy

- Each package gets a `launch/` dir with `.launch.py` files mirroring the old names.
- `swarm_exploration.launch.py` accepts `drone_num:=4` and generates per-drone nodes in a Python loop; this replaces the copy-pasted per-drone blocks in the XML.
- `simulator.xml`, `single_drone_planner.xml`, `single_drone_exploration.xml` become included sub-launch files (`IncludeLaunchDescription`) with passable args.
- Realworld launches (`*_realworld.launch.py`) gate on `inf_uwb_ros` — if the package isn't on the `AMENT_PREFIX_PATH`, fail loudly with a message pointing at the vendoring step.

## Verification plan

Do parity testing against the ROS 1 Noetic container, same environment, same `drone_num`:

1. **Build gate.** `colcon build --symlink-install` from `ros2_full_code/` — zero errors, warnings welcome.
2. **Per-wave smoke tests.**
   - Wave 1: `ros2 interface show quadrotor_msgs/msg/PositionCommand` prints fields.
   - Wave 3: launch `so3_quadrotor_simulator` standalone, publish a position command manually, observe odom.
   - Wave 4: launch `plan_env` standalone with a static `.pcd`, observe ESDF topic.
   - Wave 6: invoke LKH TSP service with a hand-crafted request, verify tour comes back.
3. **Full demo parity.** Launch `swarm_exploration.launch.py` with the pillar env + 4 drones. Trigger with the `2D Goal Pose` tool in RViz 2. Compare against the Noetic container behavior:
   - Time-to-first-allocation within ~2× of ROS 1 baseline.
   - Drones visit frontiers in a similar allocation pattern (not identical — LKH is stochastic — but pair-opt allocation times should be comparable).
   - No `Path inconsistent` / `Larger cost after reallocation` messages after trigger.
4. **Regression harness.** Capture a `rosbag2` of `/drone_*/odom` + `/drone_*/planning/travel_traj` for 120 s of exploration; compute total exploration time and swept volume. Compare against a same-settings ROS 1 bag.
5. **Realworld launches** — build-only verification (they require hardware + `inf_uwb_ros`).

## Risks & mitigations

| Risk | Mitigation |
|------|------------|
| **Executor semantics drift** — FSM nested callbacks block or deadlock under ROS 2's default single-threaded executor. | Use `MultiThreadedExecutor`; split callbacks into `MutuallyExclusive` vs `Reentrant` callback groups. Budget a day for this in Wave 7. |
| **RViz 2 plugin rewrite scope creep.** | Required, not deferrable. Budget the 2–4 days in Wave 3.5 up front; port plugins one-by-one against stock `rviz_common::Display` base classes. If the timeline slips, parallelize plugin work with Wave 4 (they share no code). |
| **Timing behavior change in B-spline optimization.** nlopt is unchanged but ROS clock sources differ (`steady_clock` vs `system_clock`). | Use `RCL_STEADY_TIME` consistently for duration math; keep `RCL_SYSTEM_TIME` only for log timestamps. |
| **Message-filter sync drops rates.** Time-sync policies changed subtle defaults. | After porting `map_ros.cpp`, log sync callback hit rate and compare to ROS 1 baseline. |
| **LKH service deadlock.** Calling `spin_until_future_complete` from inside another callback deadlocks single-threaded executors. | Put LKH client in a reentrant callback group; or convert to async-only with a `wait_for` timeout and explicit re-scheduling. |
| **CUDA depth-render path rot.** Upstream CMake path in `local_sensing` assumes `compute_61` and CUDA < 11 macros. | Keep `ENABLE_CUDA=OFF` in the first ROS 2 pass; gate any CUDA fix as a follow-up. |
| **Two `quadrotor_msgs` trees confuse the build.** | Delete the nested `uav_simulator/Utils/multi_map_server/quadrotor_msgs/` copy in the ROS 2 tree from the start. |
| **External `inf_uwb_ros` dep for realworld launches.** | Mark realworld launches as gated; don't block sim port on them. |

## Deliverables

- [ros2_full_code/](ros2_full_code/) — complete colcon workspace, `colcon build` green.
- `ros2_full_code/README_ROS2.md` — build + run instructions, parity notes, list of known behavioural differences.
- `docker-ros2/` — reproducible build image (Humble desktop + LKH + nlopt + armadillo).
- A one-command demo: `ros2 launch exploration_manager swarm_exploration.launch.py`.
- An updated [CLAUDE.md](CLAUDE.md) noting that ROS 2 is now the default path and Noetic Docker is reference-only.

## Rough effort bands

Assuming one engineer working end-to-end, no parallelization:

- **Waves 0–1 (scaffolding + IDL):** 1–2 days.
- **Wave 2 (headers/math):** 1 day.
- **Wave 3 (sim stack, minus rviz_plugins):** 3–5 days. `so3_control` nodelet + `so3_disturbance_generator` dyn-recfg add a day each.
- **Wave 3.5 (rviz_plugins):** 2–4 days standalone; can ship without initially.
- **Wave 4 (planning foundations):** 3–5 days. `plan_env` message_filters is the tricky one.
- **Wave 5 (sensing):** 2 days.
- **Wave 6 (planner orchestration + LKH services):** 3–4 days.
- **Wave 7 (exploration_manager FSMs):** 3–5 days including executor/callback-group tuning.
- **Wave 8 (launch + RViz config + integration):** 2–3 days.
- **Verification + parity pass:** 3 days.

**Total: ~4–6 engineer-weeks** for the full port with parity testing. The simulator demo alone (Waves 0–3 + 6 + 7 + sim launches, skipping multi_map_server/waypoint_generator/rviz_plugins) would be ~2.5 weeks.

## Open questions before starting

1. Is a ROS 2 Docker image desired from day one, or is native-on-host the dev target? (Recommend both — dev on host for speed, Docker for reproducibility.)
2. RViz 2 plugin port — blocking for any downstream work, or deferrable?
3. Target a bit-identical parity pass, or is "qualitatively similar swarm behavior" acceptable? (LKH is stochastic so bit-parity is impossible; qualitative parity is the realistic bar.)
4. Will `inf_uwb_ros` be ported as part of a separate effort, or vendored into `ros2_full_code/src/` from an existing ROS 2 fork?
