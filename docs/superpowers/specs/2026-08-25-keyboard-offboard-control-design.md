# Keyboard Offboard Control — Design Spec

**Date:** 2026-08-25
**Author:** alphaone
**Status:** Approved

## Problem

The README references an `offboard_control` package with a `drone_kinematic`
node that publishes PX4 offboard setpoints over MicroXRCE-DDS. The source for
that package was never committed — only stale `build/offboard_control/`
artifacts remain. There is currently **no drone control node** in the repo.

This spec defines a new ROS2 package that lets the operator fly the drone in
the SITL simulation with a keyboard by streaming PX4 offboard velocity
setpoints.

## Goals

- Fly the x500_depth SITL drone interactively from a terminal.
- Velocity setpoint control (hold key = continuous motion).
- Discrete commands: arm, disarm, takeoff, land, enter offboard, hold.
- Match the Python `ament_python` conventions already used by `gng_node`
  and `logger_stats`.

## Non-Goals

- Attitude / manual / RC-style control.
- Kill switch / flight termination.
- Hardware RC override.
- Changes to the segmentation pipeline or any other existing package.
- A launch file (run directly via `ros2 run`).

## Architecture

Single rclpy node, `keyboard_offboard_node`, with three collaborating units
in one module file:

```
┌─────────────┐   keys    ┌────────────────────┐  setpoints   ┌────────┐
│ KeyboardReader│────────▶│ OffboardController │─────────────▶│  PX4   │
│ (termios     │  state   │ (Node + 20Hz Timer)│ Trajectory-  │ via    │
│  thread)     │          │                    │ Setpoint,    │ MicroX │
└─────────────┘           │                    │ Offboard-    │ RCE-DDS│
                          │ Commands: arm/     │ ControlMode, │        │
┌─────────────┐  status   │ takeoff/land/mode  │ VehicleCmd   └────────┘
│VehicleStatus│──────────▶│                    │
│VehicleOdom  │  yaw+alt  └────────────────────┘
└─────────────┘
```

### Units

- **`KeyboardReader`** — background `threading.Thread` (daemon). Uses
  `termios`/`tty` to set the terminal to raw, non-blocking mode. Maintains
  a `set` of currently-held movement keys behind a `threading.Lock`.
  `get_held()` returns a snapshot. `pop_action()` returns a single
  discrete keypress (arm/takeoff/land/mode) or `None`. `restore_terminal()`
  is registered via `atexit` so Ctrl+C never leaves the shell broken.

- **`OffboardController`** (the `rclcpp.Node` equivalent — an `rclpy.Node`)
  — owns the ROS interface:
  - Subscribers: `/fmu/out/vehicle_status` (cache `arming_state`,
    `nav_state`), `/fmu/out/vehicle_odometry` (cache yaw + current z).
  - Publishers: `/fmu/in/offboard_control_mode`,
    `/fmu/in/trajectory_setpoint`, `/fmu/in/vehicle_command`.
  - `Timer` at `offboard_rate` (default 20 Hz): always publishes
    `OffboardControlMode{velocity=True}` then a `TrajectorySetpoint`
    whose velocity is the body→NED-rotated sum of held keys, with
    `yawspeed` from yaw keys. Position/accel/jerk fields are NaN-filled
    so PX4 only acts on velocity + yawspeed.
  - Takeoff sub-state: temporarily switches `OffboardControlMode` to
    `position=True` and sends a z-target setpoint; monitors
    `VehicleOdometry.position[2]` until within 0.2 m, then reverts to
    velocity streaming.
  - Land: publishes `VEHICLE_CMD_NAV_LAND` and stops streaming (PX4
    auto-land owns the descent).
  - Mode switch `O`: only sends `VEHICLE_CMD_SET_NAV_STATE` with
    nav_state=`NAVIGATION_STATE_OFFBOARD` (14) after the timer has
    streamed ≥ 10 setpoints (PX4 requirement). `H` sets
    `NAVIGATION_STATE_AUTO_LOITER` (4).

- **Pure helpers** (module-level functions, unit-tested):
  - `yaw_from_quaternion(q: list[float]) -> float` — extract yaw (rotation
    about NED z) from a `[w, x, y, z]` quaternion.
  - `body_to_ned(vx_body, vy_body, yaw) -> tuple[float, float]` — rotate a
    body-frame 2D velocity into NED using the current yaw.
  - `build_vehicle_command(cmd, p1=0.0, p2=0.0, p3=0.0, p4=0.0, p5=0.0, p6=0.0, p7=0.0) -> VehicleCommand`
    — fill the standard PX4 command envelope (`target_system=1`,
    `target_component=1`, `source_system=255`, `source_component=0`,
    `from_external=False`, `confirmation=0`).
  - `nan_filled_trajectory(vx, vy, vz, yawspeed) -> TrajectorySetpoint` —
    return a `TrajectorySetpoint` with position/accel/jerk set to NaN and
    only velocity + yawspeed populated.

## Frame Handling

`TrajectorySetpoint.velocity` is **NED**. Movement keys are **body-frame**
(intuitive: W = forward along the drone's heading). Body→NED rotation uses
the current yaw extracted from `VehicleOdometry.q`:

```
vx_ned =  vx_body * cos(yaw) - vy_body * sin(yaw)
vy_ned =  vx_body * sin(yaw) + vy_body * cos(yaw)
```

Altitude keys (R/F) write directly to `velocity[2]` (NED z, up = negative).
`yawspeed` carries the yaw rate from Q/E so PX4 rotates the body.

## Keymap

| Hold (velocity)            | Discrete (press once)            |
|----------------------------|----------------------------------|
| W / S — forward / back     | I — arm                          |
| A / D — strafe left / right| K — disarm                       |
| R / F — up / down          | T — takeoff (to `takeoff_alt`)   |
| Q / E — yaw CCW / CW       | L — land (`VEHICLE_CMD_NAV_LAND`)|
| Space — hover (zero vel)   | O — enter offboard (nav_state=14)|
|                            | H — hold (`AUTO_LOITER`, state=4)|

## Offboard Engagement Sequence

PX4 requires streaming setpoints **before** it will accept offboard mode.
The node's timer therefore streams zero-velocity `TrajectorySetpoint`s from
startup. Pressing `O` then sends `VEHICLE_CMD_SET_NAV_STATE` with
nav_state=14. Pressing `I` arms. Takeoff (`T`) temporarily switches to
position setpoint mode with z target = `-(takeoff_alt)`, monitors
`VehicleOdometry.position[2]` until `|diff| < 0.2 m`, then returns to
velocity streaming. Land (`L`) sends `VEHICLE_CMD_NAV_LAND`; PX4 auto-land
takes over and the node stops streaming.

## Files to Create

```
src/keyboard_offboard_control/
├── package.xml
├── setup.py
├── resource/keyboard_offboard_control
└── keyboard_offboard_control/
    ├── __init__.py
    └── keyboard_offboard_node.py
src/keyboard_offboard_control/test/test_helpers.py
```

## Parameters (ROS params with defaults)

- `linear_speed` — `0.8` m/s (magnitude of horizontal & vertical velocity)
- `yaw_speed` — `0.5` rad/s
- `takeoff_alt` — `2.5` m
- `offboard_rate` — `20` Hz

Topic names are **constants** (choice A), matching the README's
`drone_kinematic` table:

- Subscribe: `/fmu/out/vehicle_odometry`, `/fmu/out/vehicle_status`
- Publish:   `/fmu/in/trajectory_setpoint`,
             `/fmu/in/vehicle_command`,
             `/fmu/in/offboard_control_mode`

## Safety

- Timer keeps streaming at 20 Hz so offboard never times out
  (`COM_OF_LOSS_T`).
- Disarm / mode-exit commands stop the timer cleanly.
- Ctrl+C restores the terminal (`termios.tcsetattr`) and publishes a final
  disarm command.
- `TrajectorySetpoint` NaN-fills uncontrolled fields so PX4 only acts on
  velocity + yawspeed.

## Testing

Unit tests (`pytest`, matching `logger_stats` test layout) for the pure
helpers:

- `yaw_from_quaternion` at identity, 90°, 180°, -90°.
- `body_to_ned` at yaw=0, π/2, π, -π/2 (verifies rotation direction).
- `build_vehicle_command` sets `from_external=False`, `target_system=1`,
  `target_component=1`, and forwards `param1..7` + `command`.
- `nan_filled_trajectory` leaves only `velocity` + `yawspeed` non-NaN.

Integration test is manual in SITL per README Terminals 1–6, then:

```bash
ros2 run keyboard_offboard_control keyboard_offboard_node
```

Press I → arm, T → takeoff, WASD/QE/RF → fly, L → land.

## Dependencies

- `rclpy`
- `px4_msgs` (in-repo)
- `std_msgs` (for completeness, though not strictly required)

No new system dependencies.
