# nepi_servo_auto_turret

This repo holds the NEPI **SVX (servo)** device category — a board-agnostic NEPI
hardware-abstraction interface for controlling a **single positional servo** — plus
the `nepi_app_servo_auto` application that coordinates servos into an auto-turret.

The SVX category is modeled directly on the existing NEPI **PTX (pan/tilt)** category
so its files drop into the `nepi_engine_ws` submodules (`nepi_interfaces`, `nepi_api`,
`nepi_rui`, `nepi_drivers`) unchanged.

## SVX design assumptions

1. **One SVX instance = one servo.** A pan/tilt turret is two SVX instances.
   Coordinating them is the application's job (`nepi_app_servo_auto`), not the interface's.
2. **Open loop.** A 3-wire PWM servo reports nothing back. Position is reported as the
   last commanded value; nothing is measured.
3. **The API speaks degrees.** Conversion from degrees to servo pulse width is a driver
   concern, not an API concern.

## Category layout (mirrors PTX)

| Layer | File | Role |
|---|---|---|
| Interfaces | `src/nepi_interfaces/msg/DeviceSVXStatus.msg` | Servo status message |
| | `src/nepi_interfaces/srv/SVXCapabilitiesQuery.srv` | Capabilities service |
| | `src/nepi_interfaces/msg/ServoLimits.msg` | Soft-limit control message (`min_deg`/`max_deg`) |
| API | `src/nepi_api/device_if_svx.py` (`SVXActuatorIF`) | Device interface class (subscribes controls, publishes status, serves capabilities) |
| | `src/nepi_api/connect_device_if_svx.py` (`ConnectSVXDeviceIF`) | Connect client for other nodes |
| RUI | `src/nepi_rui/NepiDeviceSVX.js` | Device selector panel |
| | `src/nepi_rui/NepiDeviceSVX-Controls.js` | Control panel (drawn per capability flags) |
| | `src/nepi_rui/NepiDeviceSVX-ImageViewer.js` | Image viewer + position slider |
| Driver | `src/nepi_drivers/svx_drivers/svx_servo_generic_*` | Generic single-servo driver **stub** |

## Control topics (subscribed by `SVXActuatorIF`)

Every control is a topic the interface subscribes to; on receipt it clamps the value
against the current soft limits, then calls the matching driver callback. If a driver
callback is `None`, the corresponding `has_*` capability flag is `False` and the control
is a no-op.

| Topic | Type | Meaning |
|---|---|---|
| `goto_position` | `std_msgs/Float32` | Absolute position (deg) |
| `goto_ratio` | `std_msgs/Float32` | Position as 0.0–1.0 of soft range |
| `set_soft_limits` | `nepi_interfaces/ServoLimits` | Software min/max (deg) |
| `set_speed_ratio` | `std_msgs/Float32` | Speed dial 0.0–1.0 |
| `set_speed_max_dps` | `std_msgs/Float32` | What ratio 1.0 means (deg/s) |
| `set_reverse_enable` | `std_msgs/Bool` | Invert the axis |
| `stop_moving` | `std_msgs/Empty` | Halt motion |
| `go_home` | `std_msgs/Empty` | Move to home position |
| `set_home_position` | `std_msgs/Float32` | Set home to a specific angle (deg) |
| `set_home_position_here` | `std_msgs/Empty` | Set home to current position |
| `reset_device` | `std_msgs/Empty` | Reset device to defaults |
| `set_spin_direction` | `std_msgs/Int32` | Spin direction (continuous servos) |

## Hardware driver status

The servo-controller board is **not yet chosen** (Pololu Maestro vs. ESP32), so
`svx_drivers/svx_servo_generic_*` ships as a documented single-servo **stub**: it brings
up the full ROS interface but performs no hardware I/O. Implementing the degree →
pulse-width conversion and board link (USB serial for the Maestro, or custom ESP32
firmware) is the developer's next task — see the `TODO(board)` markers in
`svx_servo_generic_node.py`. No serial/USB dependency is added until the board is chosen.

See `docs/SVX Requirements.md` for the authoritative API specification and
`docs/SVX-Category.md` for the category overview.
