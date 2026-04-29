# Migration notes

Running log of non-mechanical changes made during the port.
Future waves of C++ porting must honor these renames.

## IDL field renames (required by ROS 2 rosidl naming rules)

ROS 2 requires field names matching `^[a-z][a-z0-9_]*$` — no leading uppercase, no trailing underscore.

| Package          | Msg              | Old field    | New field    |
|------------------|------------------|--------------|--------------|
| `plan_env`       | `ChunkData`      | `voxel_occ_` | `voxel_occ`  |
| `quadrotor_msgs` | `Gains`          | `Kp`         | `kp`         |
| `quadrotor_msgs` | `Gains`          | `Kd`         | `kd`         |
| `quadrotor_msgs` | `Gains`          | `Kp_yaw`     | `kp_yaw`     |
| `quadrotor_msgs` | `Gains`          | `Kd_yaw`     | `kd_yaw`     |
| `quadrotor_msgs` | `SO3Command`     | `kR`         | `k_r`        |
| `quadrotor_msgs` | `SO3Command`     | `kOm`        | `k_om`       |

When porting C++ that reads/writes these fields, `grep -rn` the old names in the
ROS 1 tree and update the corresponding ROS 2 sources.

## Type substitutions

| ROS 1                           | ROS 2                                  |
|---------------------------------|----------------------------------------|
| `Header header`                 | `std_msgs/Header header` (explicit)    |
| `time start_time`               | `builtin_interfaces/Time start_time`   |
