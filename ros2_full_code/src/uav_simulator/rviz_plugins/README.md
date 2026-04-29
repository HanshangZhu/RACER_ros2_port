# rviz_plugins — ROS 2 port status

## Ported

- **`rviz_plugins/Goal3DTool`** — 3D nav-goal drag tool.
  - Left-click + drag on ground plane: set x, y, and yaw.
  - Hold right mouse while dragging: raise/lower z (emits intermediate arrows).
  - Emits `geometry_msgs/msg/PoseStamped` on the topic configured in the tool property (default: `goal`).
  - Referenced by [so3_quadrotor_simulator/config/rviz.rviz](../so3_quadrotor_simulator/config/rviz.rviz).

## Sidelined (original sources kept as `*.ros1` for reference)

- **`ProbMapDisplay`**, **`MultiProbMapDisplay`**, **`AerialMapDisplay`** — custom `rviz::Display` subclasses backed by `multi_map_server` messages. The ported swarm demo ([exploration_manager/resource/exploration.rviz](../../swarm_exploration/exploration_manager/resource/exploration.rviz)) uses only stock `rviz_default_plugins/*` displays, so these are not in the critical path.
- **`GameLikeInput`** — keyboard/range-constrained goal tool. The swarm demo triggers the FSM via [scripts/trigger_swarm.sh](../../../scripts/trigger_swarm.sh) (one-shot `ros2 topic pub` on `/move_base_simple/goal`), so this isn't required either.

## How to port the sidelined ones later

Each file follows the same ROS 1 → rviz2 mapping used for `goal_tool.cpp`:

| ROS 1 rviz API | rviz2 equivalent |
|---|---|
| `#include <rviz/display.h>` | `#include <rviz_common/message_filter_display.hpp>` or `#include <rviz_common/display.hpp>` |
| `rviz::Display` | `rviz_common::Display` (non-templated) or `rviz_common::MessageFilterDisplay<T>` |
| `rviz::Properties*` | `rviz_common::properties::*` |
| `update_nh_` | `context_->getRosNodeAbstraction().lock()->get_raw_node()` |
| `pluginlib::ClassList` macro base | `rviz_common::Tool` / `rviz_common::Display` |

Rename each `.ros1` file back to `.cpp`/`.h`, apply the mechanical transforms, and add to `CMakeLists.txt` + `plugin_description.xml`.
