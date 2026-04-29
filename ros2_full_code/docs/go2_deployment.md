# Deploying RACER on a Unitree Go1 / Go2

This guide covers running the RACER exploration stack on a real Unitree quadruped. The same launch file works for Go1 and Go2 — the only differences are `stand_height` (Go1 ≈ 0.32 m, Go2 ≈ 0.30 m) and the specific SDK topic names.

## Architecture

```
        ┌──────────────────────────┐          ┌─────────────────────────┐
        │  Unitree Go1 / Go2       │          │  Host PC (this repo)    │
        │  (onboard Jetson/NUC)    │          │                         │
        │                          │          │                         │
        │ ┌──────────────────────┐ │   DDS    │ ┌─────────────────────┐ │
        │ │ unitree_ros2 /       │◀┼──────────┼─│ real_go2.launch.py  │ │
        │ │ go2_ros2_sdk         │ │ /cmd_vel │ │  ├─ exploration_node│ │
        │ │                      │ │──────────▶ │  ├─ traj_server     │ │
        │ │ publishes /odom      │ │  /odom   │ │  ├─ cmd_to_twist    │ │
        │ │ subscribes /cmd_vel  │ │──────────▶ │  └─ tsp_server      │ │
        │ └──────────────────────┘ │          │ └─────────────────────┘ │
        │                          │          │         ▲               │
        │ ┌──────────────────────┐ │  /depth  │         │               │
        │ │ RealSense D435 /     │─┼──────────┼─────────┘               │
        │ │ Livox L1 (Go2)       │ │          │                         │
        │ └──────────────────────┘ │          │                         │
        └──────────────────────────┘          └─────────────────────────┘
```

## Prerequisites

1. **Unitree ROS 2 SDK** cloned into your workspace:
   - Go2: [unitreerobotics/unitree_ros2](https://github.com/unitreerobotics/unitree_ros2) or [abizovnuralem/go2_ros2_sdk](https://github.com/abizovnuralem/go2_ros2_sdk).
   - Go1: the legacy `unitree_legged_sdk` doesn't ship a ROS 2 wrapper — use [UnitreeRoboticsJapan/go1_ros2](https://github.com/snt-arg/go1-ros2) or wrap `unitree_legged_sdk` yourself.

2. **CycloneDDS** for low-latency DDS across Ethernet:
   ```bash
   sudo apt install ros-humble-rmw-cyclonedds-cpp
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   ```
   Both host and robot need the same `ROS_DOMAIN_ID` and a `CYCLONEDDS_URI` pointing to the robot's network interface.

3. **Depth camera driver** running on the robot (for D435, `realsense2_camera`; for Go2 with native camera, enable the SDK's image service).

## Calibration steps (first time)

### 1. `stand_height`

With the robot in its trot stance, measure the base-link height above ground:
```bash
ros2 topic echo --once /odom | grep -A3 position
```
The reported `position.z` is your `stand_height`. Set this via the launch arg.

### 2. Camera intrinsics

If using a custom-calibrated D435, override cx/cy/fx/fy — defaults in the launch file come from a factory-calibrated unit.

### 3. World origin alignment

The planner assumes `odom` frame == `world` frame. If your SDK publishes odometry in a different frame (e.g. `base_odom`), add a static_transform_publisher:
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world base_odom
```

## Running

### Terminal 1 — Unitree SDK (on the robot or routed to the host)
Follow the Unitree SDK docs. Confirm:
```bash
ros2 topic list | grep -E 'odom|cmd_vel|depth'
```
and `ros2 topic hz /odom` shows a steady rate (typically 50–200 Hz).

### Terminal 2 — RViz on host
```bash
cd ros2_full_code && source install/setup.bash
ros2 launch exploration_manager rviz.launch.py
```

### Terminal 3 — RACER planner + Twist bridge
```bash
ros2 launch exploration_manager real_go2.launch.py \
    drone_id:=1 \
    stand_height:=0.30 \
    odom_topic:=/odom \
    depth_topic:=/camera/depth/image_rect_raw
```

### Terminal 4 — trigger exploration
Stand the robot somewhere safe. When ready:
```bash
./scripts/trigger_swarm.sh --quadruped
```

## Safety checklist

**Before every run:**

- [ ] Emergency stop button mapped on the Unitree remote or a host-side kill switch (e.g. `ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'` killed repeatedly will make the robot halt).
- [ ] `v_fwd_max` lowered to `0.5 m/s` for the first run — raise once you've seen the planner behave.
- [ ] Physical safety radius cleared (~2 m around the robot during initial tests).
- [ ] `stand_height` measured, not assumed.
- [ ] Depth camera pointing forward, not straight down.

**During the run:** if the robot's commanded path deviates from the planned trajectory by > 0.5 m (visible in RViz), something is mistuned — hit e-stop and check the `cmd_to_twist` clipping caps.

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| Robot doesn't move after trigger | `/cmd_vel` not reaching SDK, or `cmd_to_twist` stalled waiting for `/odom` | `ros2 topic echo /cmd_vel` — should publish zeros until trigger, body-frame velocities after. Check DDS/network. |
| Plans but drifts off course | Twist clipping too aggressive for the planned trajectory | Increase `v_lat_max` slightly, or escalate to Phase 3 (non-holonomic planner). |
| `exploration_node` segfaults at first tick | No depth cloud yet — planner calls `findGlobalTourOfGrid` against empty map | Delay `trigger_swarm.sh` by ~3 s after launch, or raise `pcl_render_node.sensing_rate`. Same bug as simulated heavy sim. |
| Goes up or down stairs | Thin z-slab not constraining the planner's vertical search enough | Tighten `sdf_map.box_min_z / box_max_z` further around `stand_height`. |

## Escalation to Phase 3 (non-holonomic planner)

Phase 2 clips holonomic trajectories post-hoc. On real hardware this typically costs 0.3–0.5 m RMS tracking error. If that's unacceptable, enable the Phase-3 non-holonomic planner (unicycle kinodynamic A*). See `PORT_STATUS.md` → "Quadruped" section.
