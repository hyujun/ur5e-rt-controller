# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Run Commands

**Prerequisites**: Ubuntu 22.04, ROS2 Humble, `realtime` group membership with `rtprio 99` / `memlock unlimited` in `/etc/security/limits.conf`.

```bash
# Automated setup (installs deps, builds, sets RT permissions)
chmod +x install.sh && ./install.sh

# Manual build (from workspace root, not repo root)
cd ~/ur_ws
colcon build --packages-select ur5e_rt_controller --symlink-install
source install/setup.bash

# Run full system (real robot)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch ur5e_rt_controller ur_control.launch.py robot_ip:=192.168.1.10

# Run with fake hardware (no robot needed)
ros2 launch ur5e_rt_controller ur_control.launch.py use_fake_hardware:=true

# Run UDP hand nodes only
ros2 launch ur5e_rt_controller hand_udp.launch.py udp_port:=50001 target_ip:=192.168.1.100 target_port:=50002
```

**Monitoring**:
```bash
ros2 topic hz /forward_position_controller/commands   # should be ~500Hz
ros2 topic echo /system/estop_status                  # true = E-STOP active
ros2 control list_controllers -v
PID=$(pgrep -f custom_controller) && ps -eLo pid,tid,cls,rtprio,psr,comm | grep $PID
```

**Manually publish a target pose**:
```bash
ros2 topic pub /target_joint_positions std_msgs/msg/Float64MultiArray "data: [0.0, -1.57, 0.0, 0.0, 0.0, 0.0]"
```

## Architecture

This is a **ROS2 Humble** package (`ament_cmake`, C++20) implementing a 500 Hz real-time position controller for a UR5e robot arm with an 11-DOF custom hand attached via UDP.

### Core Design: Strategy Pattern + Multi-threaded Executors

`RTControllerInterface` (`include/ur5e_rt_controller/rt_controller_interface.hpp`) is the abstract base for all controllers. All virtual methods are `noexcept` — a hard RT safety requirement (an exception in a 500 Hz loop terminates the process). The key data types defined here are used throughout the codebase:
- `RobotState`: 6-DOF joint positions/velocities + TCP position
- `HandState`: 11-DOF motor positions/velocities/currents + 44 sensor values
- `ControllerState`: combined robot + hand state passed to `Compute()`
- `ControllerOutput`: 6 robot commands + 11 hand commands

`PDController` (`include/ur5e_rt_controller/controllers/pd_controller.hpp`) is the active implementation. On E-STOP it drives toward `kSafePosition = [0, -1.57, 1.57, -1.57, -1.57, 0]` rad. `PController` exists as a simpler alternative.

### Main Node: `CustomController` (`src/custom_controller.cpp`)

The entire executable lives in this one file. It creates **4 `SingleThreadedExecutor`s**, each running in a dedicated `std::thread` with RT scheduling applied via `ApplyThreadConfig()`:

| Executor / Thread | Callback Group | CPU Core | Scheduler | Priority | What runs here |
|---|---|---|---|---|---|
| `rt_executor` / `t_rt` | `cb_group_rt_` | Core 2 | SCHED_FIFO | 90 | `ControlLoop()` (500Hz), `CheckTimeouts()` (50Hz E-STOP watchdog) |
| `sensor_executor` / `t_sensor` | `cb_group_sensor_` | Core 3 | SCHED_FIFO | 70 | `/joint_states`, `/target_joint_positions`, `/hand/joint_states` subscribers |
| `log_executor` / `t_log` | `cb_group_log_` | Core 4 | SCHED_OTHER | nice -5 | `DataLogger` CSV writes |
| `aux_executor` / `t_aux` | `cb_group_aux_` | Core 5 | SCHED_OTHER | 0 | E-STOP status publisher |

`mlockall(MCL_CURRENT | MCL_FUTURE)` is called at startup to prevent page faults. Shared state between the sensor thread and RT thread is protected by three separate mutexes (`state_mutex_`, `target_mutex_`, `hand_mutex_`).

### UDP Hand Protocol

`HandUdpReceiver` (`include/ur5e_rt_controller/hand_udp_receiver.hpp`) uses `std::jthread` (C++20 cooperative cancellation) to receive packets on port 50001. Each packet is 77 `double`s (616 bytes): 11 motor positions + 11 velocities + 11 currents + 44 sensor values.

`HandUdpSender` sends 4 `double`s (normalized 0.0–1.0 motor commands) to port 50002.

### E-STOP System

`CheckTimeouts()` runs at 50 Hz. If `/joint_states` is not received for >100ms, `PDController::TriggerEstop()` is called (sets `estopped_` atomic flag). If `/hand/joint_states` is not received for >200ms, `SetHandEstop(true)` is called separately. Both flags use `std::atomic<bool>` for safe cross-thread access between the RT thread and the 50 Hz watchdog.

To disable hand E-STOP (when no hand is connected): set `enable_estop: false` or `hand_timeout_ms: 0` in `config/ur5e_rt_controller.yaml`.

### Adding a Custom Controller

Inherit from `RTControllerInterface`, implement `Compute()`, `SetRobotTarget()`, `SetHandTarget()`, and `Name()` — all must be `noexcept`. Then replace `PDController` in `custom_controller.cpp` line ~41. No CMakeLists changes needed (controllers are header-only or linked into the same executable).

### Key Constants (from `rt_controller_interface.hpp`)

- `kNumRobotJoints = 6`
- `kNumHandJoints = 11`
- `kNumHandSensors = 44` (4 sensors × 11 joints)

## RT Permissions (required for SCHED_FIFO)

```bash
sudo groupadd realtime
sudo usermod -aG realtime $USER
echo "@realtime - rtprio 99" | sudo tee -a /etc/security/limits.conf
echo "@realtime - memlock unlimited" | sudo tee -a /etc/security/limits.conf
# Must log out and log back in
ulimit -r  # should print 99
```

If `ApplyThreadConfig()` fails, the node logs `[WARN] Thread config failed` and continues without RT scheduling (increased jitter). The controller still functions but 500 Hz timing is not guaranteed.
