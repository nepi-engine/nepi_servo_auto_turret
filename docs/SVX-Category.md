# SVX (Servo) Device Category

The SVX category is the NEPI hardware-abstraction layer for a **single positional
servo**. It follows the same three-tier / four-layer shape as every other NEPI device
category (IDX cameras, PTX pan/tilts, LSX lights, NPX nav, RBX robots) and is derived
directly from the PTX (pan/tilt) category, collapsed from two axes to one.

`SVX` is the three-letter category prefix; the ROS namespace for a servo node is
`<device>/svx/...`.

## Why SVX exists separately from PTX

PTX assumes a coordinated pan **and** tilt mechanism with position feedback, timed jog
moves, and speed smoothing. A bare 3-wire PWM servo has none of that:

- it is a **single axis**, so every dual-axis PTX field, control, and callback collapses to one;
- it is **open loop** — it reports nothing, so "current position" is just the last commanded value;
- it has **no hardstops to calibrate** and **no navigation role**, so calibration and NavPose are dropped.

A pan/tilt turret is therefore **two SVX instances**; coordinating them belongs to the
application layer (`nepi_app_servo_auto`), not to `SVXActuatorIF`.

## Interface surface

### Status — `DeviceSVXStatus.msg`
Device identity; `settings_topic`/`save_data_topic`; capability flags
(`has_absolute_positioning`, `has_adjustable_speed`, `has_limit_controls`, `has_homing`,
`has_set_home`, `has_spin`); `move_decimal_place`/`report_decimal_place`;
`position_now_deg`/`position_goal_deg` and their `_ratio` forms (last-commanded, open loop);
`min_softstop_deg`/`max_softstop_deg`; `reverse_enabled`; `spin_direction`/`is_spinning`;
`speed_max_dps`/`speed_ratio`; `home_pos_deg`; `error_msgs`.

Speed is reported as **both** a ratio and a max (`speed_ratio` + `speed_max_dps`) so the
UI can show "0.5 (= 30°/s)". Soft limits are included so the UI knows the slider bounds.

### Capabilities — `SVXCapabilitiesQuery.srv`
`device_*`, `mode`, and the flags the UI uses to decide which controls to draw:
`has_absolute_positioning` (from `gotoPositionCb`), `has_adjustable_speed`
(`setSpeedRatioCb` + `getSpeedRatioCb`), `has_limit_control` (`getSoftLimitsCb`),
`has_homing` (`goHomeCb`), `has_set_home` (`setHomePositionHereCb`), `has_spin_control`,
`has_goto_control`, `has_stop_control`, and `max_speed_dps`.

### Controls
The 12 Figure-5 control topics listed in the repo README, each clamped against the
current soft limits by `SVXActuatorIF` before the driver callback runs.

### `SVXActuatorIF` construction (Figure 1)
The device node instantiates `SVXActuatorIF` with the servo's callbacks. Passing `None`
for a callback disables the matching capability and makes its control a no-op — this is
how one interface serves servos with different feature sets.

## Driver pattern

Both SVX drivers follow the standard NEPI three-tier driver pattern (discovery + node +
params yaml). The chosen board is the **Pololu Micro Maestro 6-Channel USB Servo
Controller**.

`svx_drivers/svx_servo_maestro_*` — the working Maestro driver:

- `svx_servo_maestro_discovery.py` — `discoveryFunction(...)` enumerates USB serial ports,
  matches the Pololu vendor ID (`0x1FFB`) and Maestro product IDs, picks each board's
  Command Port (lowest USB interface), and launches one node per configured servo
  `channel`.
- `svx_servo_maestro_node.py` — reads `~drv_dict`, opens the Command Port with pyserial,
  and registers with `SVXActuatorIF`. Hardware callbacks convert degrees ↔ pulse width
  (Maestro targets are in quarter-microseconds) and issue the Maestro
  [serial commands](https://www.pololu.com/docs/0J40/5.e) (Set Target/Speed/Accel, Get
  Position/Errors, Go Home). Compact protocol by default; Pololu protocol (device number)
  for daisy chains. Every serial transaction is wrapped in an in-process lock plus an
  advisory `fcntl.flock` on the port, so multiple channel nodes can share one board's USB
  port without interleaving bytes.
- `svx_servo_maestro_params.yaml` — driver metadata (`type: SVX`) plus discovery OPTIONS:
  `channels`, `protocol`, `device_number`, `command_port_index`, `baud_rate`,
  `pulse_min_us`/`pulse_max_us`, `min_deg`/`max_deg`, `accel_units`.

`svx_drivers/svx_servo_generic_*` — retained board-agnostic **stub** (no hardware I/O),
kept as the starting point for a future board (e.g. a custom-firmware ESP32). Its
`TODO(board)` callbacks are implemented per-board when that board is chosen.

**Reverse axis:** `SVXActuatorIF` applies the reverse-axis conversion (and soft-limit
clamping and ratio math) *before* it calls a driver callback, and re-applies it when it
reads position back. The Maestro driver therefore keeps a single pure linear degree ↔
pulse-width map with no reverse logic of its own — adding reverse handling in the driver
would double-invert.

**Validation status:** the Maestro driver is written against the documented protocol but
has **not** been hardware-validated (no Maestro attached in this environment). Bench
validation once a board is available: confirm the Command Port is selected (flip
`command_port_index` if commands get no response), verify `pulse_min_us`/`pulse_max_us`
against the actual servo travel, and check that a two-channel turret runs as two coexisting
nodes on one port.

## Security / trust boundary

The trust boundary is the set of ROS control topics. Any commanded position is clamped
to the current soft limits inside `SVXActuatorIF` before it reaches a driver callback, so
a bad `goto_position` cannot be passed raw to hardware. The category needs no network
access beyond ROS and stores no user-identifiable data.
