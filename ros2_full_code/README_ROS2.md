# RACER — Native ROS 2 Port

This is the **standalone ROS 2 Humble port** of RACER. It lives independently of the original ROS 1 tree under [`swarm_exploration/`](../swarm_exploration/) and [`uav_simulator/`](../uav_simulator/), which are preserved for reference only.

## Prerequisites

- Ubuntu 22.04 + ROS 2 Humble desktop-full
- `colcon`, `rosdep` initialized
- `nlopt` v2.7.1 installed to `/usr/local/lib/libnlopt.so`
- `LKH` 3.0.6 at `/usr/local/bin/LKH`
- `libarmadillo-dev`, `libompl-dev`, `libdw-dev`, `libpcl-dev`

## Build

```bash
cd ros2_full_code
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run the simulation swarm demo

```bash
ros2 launch exploration_manager rviz.launch.py          # terminal 1
ros2 launch exploration_manager swarm_exploration.launch.py  # terminal 2
# Click "2D Goal Pose" in RViz 2 to trigger.
```

## Port status

See [../ROS2_PORT_PLAN.md](../ROS2_PORT_PLAN.md) for the full plan.
Per-wave status is tracked in the top-level TODO.
