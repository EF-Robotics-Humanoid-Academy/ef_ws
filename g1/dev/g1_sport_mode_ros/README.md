# G1 Sport Mode ROS Driver

This ROS 2 package bridges `geometry_msgs/msg/Twist` commands from `/cmd_vel`
to Unitree G1 sport-mode locomotion. It includes a native Unitree SDK Python
bridge and a separate C++ API-request publisher.

## Python SDK bridge

`g1_sport_ros.py` calls the Unitree SDK directly. The multiprocess variant,
`g1_sport_multiprocess.py`, keeps SDK calls in a child process so a native SDK
failure does not take down the ROS executor.

Both Python bridges:

- clamp linear and angular speeds;
- ignore non-finite values and apply a small configurable deadband;
- stop the robot when `/cmd_vel` has not been refreshed before the watchdog
  timeout; and
- stop the robot during shutdown.

Useful parameters:

| Parameter | Default | Purpose |
| --- | ---: | --- |
| `interface` | `eth0` | Network interface used by the Unitree SDK |
| `sdk_domain_id` | `0` | Native SDK DDS domain |
| `command_timeout_sec` | `0.5` | Maximum age of a motion command |
| `max_linear_speed` | `0.5` | Absolute limit for `linear.x` and `linear.y` |
| `max_angular_speed` | `1.0` | Absolute limit for `angular.z` |
| `deadband` | `0.02` | Values below this magnitude become zero |

## Build and run

From the ROS 2 workspace root:

```bash
colcon build --packages-select g1_sport_mode_ros
source install/setup.bash
```

The ROS graph and the native SDK use separate DDS domains. Run ROS tools in
the same non-zero domain, and leave the SDK domain at zero unless the robot
setup requires another value:

```bash
unset CYCLONEDDS_HOME
ROS_DOMAIN_ID=1 ros2 run g1_sport_mode_ros g1_sport_ros.py \
  --ros-args \
  -p interface:=eth0 \
  -p command_timeout_sec:=0.5
```

Then publish commands from the same ROS domain:

```bash
ROS_DOMAIN_ID=1 ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

For the multiprocess bridge, run `g1_sport_multiprocess.py` instead. Do not
run both Python bridges at the same time because each one can command the
robot independently.

## C++ API bridge

The `g1_sport_mode_driver` executable publishes Unitree API request messages:

- subscribes to `/cmd_vel` (`geometry_msgs/msg/Twist`);
- publishes `/api/sport/request` (`unitree_api/msg/Request`); and
- maps `linear.x` → `vx`, `linear.y` → `vy`, and `angular.z` → `vyaw`.

Use only one bridge for a robot at a time. The C++ bridge is intended for
systems where another component owns the final safety and command-timeout
policy; the Python bridges include that policy themselves.

## Safety

Test with the robot lifted or in a clear area, start with conservative limits,
and keep an independent emergency stop available. A stale `/cmd_vel` command
is stopped automatically by the Python bridges, but this is not a replacement
for the robot's hardware safety controls.

If the SDK cannot initialize, verify the interface, DDS domain separation,
`CYCLONEDDS_HOME`, robot sport mode, and network connectivity. Never expose
the command topic to an untrusted ROS domain.
