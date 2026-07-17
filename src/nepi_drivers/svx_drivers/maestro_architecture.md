# Pololu Maestro SVX Driver Architecture

This document explains how the `svx_servo_maestro_*` driver works, in detail. It sits next
to the three files it describes:

- `svx_servo_maestro_discovery.py`
- `svx_servo_maestro_node.py`
- `svx_servo_maestro_params.yaml`

If you only need the short version, read the "Hardware driver status" section of the repo
`README.md`. This document is for a developer who needs to modify the driver, port it to a
different board, or debug it on hardware.


## 1. What the hardware is

The board is a **Pololu Micro Maestro 6-Channel USB Servo Controller**. It is a small USB
device running Pololu factory firmware. You plug it into the NEPI device by USB, plug up to
six 3-wire hobby servos into its channel headers, and it generates the PWM pulses that drive
those servos. The NEPI device never touches PWM directly (its GPIO is enclosed and not
exposed), so the Maestro is the bridge between USB and servo pulses.

Over USB the Maestro presents itself as **two virtual serial ports**:

- the **Command Port**, which accepts the native serial command set, and
- the **TTL Port**, used for its UART/TTL pass-through features.

This driver only ever uses the Command Port. On Linux both ports appear as `/dev/ttyACM*`
entries that share the same USB vendor ID, product ID, and serial number. They differ only
by USB interface number, which is how discovery tells them apart (see section 4).

The Maestro serial protocol is documented by Pololu at
<https://www.pololu.com/docs/0J40/5.e>. Everything this driver sends and receives comes from
that page.


## 2. Where this fits in NEPI

This is a standard NEPI three-tier device driver in the **SVX (servo)** category. The rule
of the SVX category is **one SVX device = one servo**. A pan/tilt turret is therefore two
SVX devices, and coordinating them is the job of the `nepi_app_servo_auto` application, not
of this driver.

```
             ROS control topics (goto_position, set_speed_ratio, ...)
                              |
                              v
                    +-------------------+
                    |  SVXActuatorIF    |   (nepi_api/device_if_svx.py)
                    |  - clamps limits  |
                    |  - reverse axis   |
                    |  - ratio math     |
                    |  - publishes      |
                    |    status + caps  |
                    +-------------------+
                              |  callbacks (degrees)
                              v
                    +-------------------+
                    | Maestro SVX node  |   (this driver)
                    | - deg <-> pulse   |
                    | - serial framing  |
                    +-------------------+
                              |  bytes over USB serial
                              v
                    +-------------------+
                    |  Pololu Maestro   |   (factory firmware)
                    +-------------------+
                              |  PWM
                              v
                            servo
```

The driver never creates ROS publishers, subscribers, or services itself. `SVXActuatorIF`
owns the entire ROS surface. The driver's whole job is to answer a small set of callbacks in
**degrees** and turn those into Maestro serial transactions. This is the hardware
abstraction boundary: applications above the IF speak degrees and never know a Maestro
exists.


## 3. The three files

| File | Role |
|---|---|
| `svx_servo_maestro_discovery.py` | Detects Maestro boards on USB and launches one node per configured servo channel. Called on a timer by `drivers_mgr`. |
| `svx_servo_maestro_node.py` | One running process per servo. Opens the Command Port, registers with `SVXActuatorIF`, converts degrees to pulse widths, and talks to the board. |
| `svx_servo_maestro_params.yaml` | Driver metadata (`type: SVX`) and the discovery `OPTIONS` a user can set in the RUI. |

The structure mirrors `svx_servo_generic_*` (the board-agnostic SVX template that ships
alongside as a stub) and the serial send/receive machinery mirrors
`ptx_sidus_ss109_serial_node.py` (the Sidus serial pan/tilt driver).


## 4. Discovery: `svx_servo_maestro_discovery.py`

`drivers_mgr` calls `discoveryFunction(available_paths_list, active_paths_list,
base_namespace, drv_dict, retry_enabled)` on a polling interval (roughly every 1 to 3
seconds). Each call does three things.

### 4.1 Read the options

Every pass re-reads the discovery `OPTIONS` from `drv_dict` (channels, protocol, device
number, command port index, baud, pulse-width endpoints, degree range, acceleration). This
means a user can change an option in the RUI and it takes effect on the next launch without
editing code.

### 4.2 Purge nodes whose board vanished

For every node this discovery has launched, it checks whether the node's serial path is
still in `available_paths_list` (the set of serial ports the manager currently sees). If the
path is gone, the node is killed and removed from tracking. This is the same
available-paths-based liveness check the Sidus serial discovery uses.

### 4.3 Find Command Ports and launch nodes

`findCommandPorts()` enumerates `serial.tools.list_ports.comports()` and keeps the entries
whose USB vendor ID is Pololu (`0x1FFB`) and whose product ID is a known Maestro product ID
(`0x0089` for the Micro Maestro 6, plus the Mini Maestro 12/18/24 IDs so the driver also
covers those boards).

Because a single board exposes two serial ports with identical VID, PID, and serial number,
the matches are grouped by serial number so that each group is one physical board. Within a
group, `interfaceSortKey()` parses the USB interface number out of `ListPortInfo.location`
(a location like `1-1.2:1.0` ends in interface `0`) and sorts ascending. The **Command
Port** is the lowest interface number. The `command_port_index` option overrides which entry
of the sorted list is treated as the Command Port, for the rare board or platform where the
ordering differs.

For each Command Port found, and for each channel in the `channels` option, discovery calls
`launchDeviceNode(path, channel, serial_number)`. That function:

1. builds a unique device name (`maestro_<port>_ch<N>`),
2. writes a per-node `drv_dict` to the ROS param server at `<node>/drv_dict`, embedding the
   port, channel, baud, device number, and serial number in `DEVICE_DICT`, and
3. calls `nepi_drvs.launchDriverNode(...)` to start the node process.

Launched nodes are tracked in `active_devices_dict`, keyed by a launch id of
`"<port>:ch<N>"`, so one board with two servos produces two independent entries that share
one path. The path is added to `active_paths_list` once, so the manager does not re-scan it,
and it is removed only after the last channel node on that path is gone.

### 4.4 Why one node per channel

Keeping "one SVX device = one servo" means each servo is an independent NEPI device with its
own status, its own soft limits, and its own controls. A pan/tilt turret is then two SVX
devices on one board, which is exactly what `nepi_app_servo_auto` expects. The cost is that
two node processes now share one physical serial port, which section 6 addresses.


## 5. Node: `svx_servo_maestro_node.py`

### 5.1 Startup sequence

1. `nepi_sdk.init_node(...)` and create the `MsgIF` logger.
2. Read `~drv_dict` from the param server. Pull the port, channel, baud, device number, and
   serial number out of `DEVICE_DICT`; pull protocol, pulse-width endpoints, degree range,
   and acceleration out of `DISCOVERY_DICT.OPTIONS`. If the pulse-width range is invalid it
   falls back to 1000 to 2000 microseconds.
3. `connect()` opens the serial port and runs a handshake (section 5.4). If it fails, the
   node shuts itself down so `drivers_mgr` can retry later.
4. Push the initial acceleration and speed to the board.
5. Construct `SVXActuatorIF`, passing the callbacks in section 5.2.
6. Start a repeating one-shot timer (`updatePositionHandler`) that polls the board position
   and error flags at `MAX_POSITION_UPDATE_RATE` (5 Hz).
7. `nepi_sdk.spin()`.

### 5.2 Callback wiring to the SVX interface

The node passes these callbacks to `SVXActuatorIF`. The interface derives each `has_*`
capability flag from which callbacks are non-None, which is how the RUI decides what
controls to draw.

| IF constructor arg | Node method | Effect |
|---|---|---|
| `stopMovingCb` | `stopMoving` | has_stop_control |
| `gotoPositionCb` | `gotoPosition` | has_absolute_positioning / has_goto_control |
| `getPositionCb` | `getPosition` | position readback |
| `setSoftLimitsCb` / `getSoftLimitsCb` | `setSoftLimits` / `getSoftLimits` | has_limit_control |
| `setSpeedRatioCb` / `getSpeedRatioCb` | `setSpeedRatio` / `getSpeedRatio` | has_adjustable_speed |
| `setSpeedMaxCb` / `getSpeedMaxCb` | `setSpeedMax` / `getSpeedMax` | what a ratio of 1.0 means in deg/s |
| `goHomeCb` | `goHome` | has_homing |
| `setHomePositionCb` / `setHomePositionHereCb` | `setHomePosition` / `setHomePositionHere` | has_set_home |
| `deviceResetCb` | `resetDevice` | reset to defaults |
| `setSpinDirection` / `getSpinDirection` | **None** | has_spin = False (positional servo) |

The spin callbacks are None on purpose. This driver targets positional servos, which have no
spin concept. See section 8 for continuous-rotation servos.

### 5.3 What the interface does before the driver is called

This matters for correctness. `SVXActuatorIF` does three transforms **before** it calls
`gotoPositionCb`, and the driver must not repeat them:

- **Soft-limit clamping.** A commanded degree value is clamped to the current soft limits.
  A bad `goto_position` can never reach the board. This is the security trust boundary.
- **Reverse axis.** If the axis is reversed, the interface multiplies the degree value by
  its reverse index (`ri = -1`) on the way in, and re-applies it on readback. The driver
  keeps a single pure linear degree-to-pulse map and does no reverse handling of its own.
  Adding reverse logic in the driver would double-invert the axis.
- **Ratio and rounding.** `goto_ratio` is converted to degrees by the interface, and values
  are rounded to `move_decimal_place`.

So by the time `gotoPosition(deg)` runs in the driver, `deg` is safe, in the driver's own
non-reversed frame, and ready to convert.

### 5.4 Connection and handshake

`connect()` opens the port with pyserial at the configured baud and a short read timeout. It
then sends **Get Errors** and expects exactly two bytes back. This doubles as a probe: the
real Command Port always answers Get Errors, so a two-byte reply confirms both that the port
is a Maestro Command Port and that the protocol and device number are correct. Anything else
(no reply, wrong length) means this is the TTL Port or a misconfigured device, and the node
refuses to come up. If discovery picked the wrong port, the node fails cleanly and the user
can flip `command_port_index`.

Note on baud: for the USB Command Port the baud rate is irrelevant because USB is not
rate-limited. The baud option exists for completeness and for TTL/UART use.


## 6. Sharing one serial port across channel nodes

Two servos on one Maestro means two node processes opening the same `/dev/ttyACM*`. On Linux
that is allowed, but if both processes wrote at the same time their bytes could interleave
and corrupt a command. The driver prevents this with **two locks** inside `send_cmd`:

1. a `threading.Lock` (`self.serial_lock`), which serializes transactions within one process,
   and
2. an advisory file lock, `fcntl.flock(fd, LOCK_EX)` on the serial device, which serializes
   transactions across processes.

Each transaction acquires both locks, writes the complete command frame, reads the fixed
number of response bytes if the command has a reply, then releases the locks. Because every
command is a short self-contained byte sequence and the lock is held for the whole
write-then-read, two channel nodes can hammer the same board without ever interleaving. If
`fcntl` is unavailable (non-Linux), the code degrades to the in-process lock only.


## 7. The Maestro protocol as used here

All commands come from the Pololu documentation. The Maestro works in units of
**quarter-microseconds** for both targets and reported positions.

### 7.1 Command set

| Meaning | Compact byte | Data |
|---|---|---|
| Set Target | `0x84` | channel, target low 7 bits, target high 7 bits |
| Set Speed | `0x87` | channel, speed low 7 bits, speed high 7 bits |
| Set Acceleration | `0x89` | channel, accel low 7 bits, accel high 7 bits |
| Get Position | `0x90` | channel, then read 2 bytes (little-endian) |
| Get Moving State | `0x93` | read 1 byte (0 stopped, 1 moving) |
| Get Errors | `0xA1` | read 2 bytes (little-endian flags, cleared on read) |
| Go Home | `0xA2` | none |

Data bytes are **7-bit**: the low byte is `value & 0x7F` and the high byte is
`(value >> 7) & 0x7F`. This is why the byte masking appears everywhere in the driver.

### 7.2 Compact vs Pololu protocol

`send_cmd(cmd_byte, payload, read_len)` frames the command in one of two ways:

- **Compact** (default): `[cmd_byte] + payload`. Works for a single controller on the port
  regardless of its configured device number.
- **Pololu**: `[0xAA, device_number, cmd_byte & 0x7F] + payload`. The command byte has its
  high bit cleared and is preceded by the frame start byte `0xAA` and the target device
  number. Use this when several Maestros are daisy-chained on one line so each command is
  addressed to a specific board. The factory default device number is 12.

`read_len` tells `send_cmd` how many response bytes to read, so the same function handles
both write-only commands (Set Target) and query commands (Get Position, Get Errors).


## 8. Unit conversions

The API speaks degrees; the Maestro speaks quarter-microseconds and speed units. The driver
is the only place that conversion happens.

### 8.1 Degrees to pulse width

A linear map is defined by four numbers from the options: `pulse_min_us`, `pulse_max_us`,
`min_deg`, `max_deg`. `min_deg` maps to `pulse_min_us` and `max_deg` maps to `pulse_max_us`.

```
frac = (deg - min_deg) / (max_deg - min_deg)
us   = pulse_min_us + frac * (pulse_max_us - pulse_min_us)     # clamped to the endpoints
target_quarter_us = round(us * 4)
```

The reverse map (`us2deg`, `target2deg`) is the exact inverse and is used by the position
readback. The clamp to the pulse-width endpoints is a second safety net below the interface's
soft-limit clamp, so a conversion can never command a pulse outside the servo's mechanical
range.

### 8.2 Speed ratio to Maestro speed units

One Maestro speed unit is a change of 0.25 microseconds every 10 milliseconds, which is
25 microseconds per second. The effective speed in degrees per second is
`ratio * speed_max_dps`. That is converted to microseconds per second using the pulse-width
span per degree, then divided by 25 to get Maestro units:

```
us_per_sec = ratio * speed_max_dps * (pulse span per degree)
units      = round(us_per_sec / 25)
```

A Maestro speed of 0 means "no speed limit" (move as fast as possible), which is not what a
ratio of 0.0 should mean, so the result is clamped to a minimum of 1 (the slowest non-zero
speed). `setSpeedMax` re-applies the current ratio so that a ratio still means the same
fraction of the new maximum.

### 8.3 Position, stop, home, reset

- **Position readback.** The 5 Hz timer calls Get Position, which returns the pulse width
  the Maestro is currently transmitting. Because the Maestro applies its own speed and
  acceleration limits, this value tracks a slew in progress rather than jumping to the goal.
  The result is cached in `position_deg`; `getPosition()` returns the cache so the interface
  status publisher stays cheap. A value of 0 means the channel is not being driven and is
  reported as "no position".
- **Stop.** For a positional servo, stop means hold. `stopMoving` reads the current pulse
  width and re-commands it as the target, which halts any in-progress slew.
- **Home.** `goHome` is `gotoPosition(home_pos_deg)`. Home is a software concept in degrees,
  set by `setHomePosition` (a specific angle) or `setHomePositionHere` (the current
  position).
- **Reset.** `resetDevice` sends the Maestro Go Home command, clears the error register, and
  restores this node's factory speed, max speed, limits, and acceleration.

### 8.4 Errors

The 5 Hz timer also calls Get Errors. Any non-zero flag word is logged as a warning
(`Maestro error flags: 0x....`). Reading the register clears it on the board, which is the
Maestro's documented behavior.


## 9. Options reference

Set in the RUI or in `svx_servo_maestro_params.yaml` under `DISCOVERY_DICT.OPTIONS`.

| Option | Meaning | Default |
|---|---|---|
| `channels` | Which Maestro channels to expose as servos. `"0"`, `"0,1"`, or `"all"` (0 to 5). One node per channel. | `0` |
| `protocol` | `Compact` (single board) or `Pololu` (daisy chain, uses device number). | `Compact` |
| `device_number` | Maestro device number, Pololu protocol only. Factory default 12. | `12` |
| `command_port_index` | Which of a board's two USB ports is the Command Port. `0` = lowest USB interface. Flip to `1` if commands get no response. | `0` |
| `baud_rate` | Serial baud. Ignored for the USB Command Port. | `9600` |
| `pulse_min_us` | Pulse width at `min_deg`. | `1000` |
| `pulse_max_us` | Pulse width at `max_deg`. | `2000` |
| `min_deg` | Degree value mapped to `pulse_min_us`. | `-90` |
| `max_deg` | Degree value mapped to `pulse_max_us`. | `90` |
| `accel_units` | Maestro acceleration limit; 0 = no limit. | `0` |


## 10. Validation status and how to bench-test

The driver is written against the documented Maestro protocol but has **not** been
hardware-validated in the development environment, because no Maestro is attached there. When
a board is available:

1. Plug in the Maestro and confirm two `/dev/ttyACM*` entries appear. Confirm the driver
   detects one board and launches a node for channel 0.
2. If the node fails its handshake, flip `command_port_index` from 0 to 1. The driver picks
   the lower USB interface as the Command Port by default, but confirm that on the actual
   board.
3. Command a few `goto_position` values and confirm the servo moves to the expected angle.
   If the travel is wrong, adjust `pulse_min_us` and `pulse_max_us` to match the servo's real
   pulse range, and `min_deg` / `max_deg` to its real travel.
4. Set `channels` to `0,1`, plug in a second servo, and confirm both channels move
   independently and that neither node reports serial corruption. This exercises the shared
   port locking.
5. Confirm `set_speed_ratio` visibly changes slew speed and that `stop_moving` halts a slew.


## 11. Extending or porting

- **A different Maestro model.** The Mini Maestro product IDs are already in the match list,
  and the channel range in discovery goes up to 24. Only `channels: all` is hardcoded to
  0 to 5 for the Micro Maestro; widen it if you use `all` on a larger board.
- **A continuous-rotation servo.** On a continuous servo the target sets speed and direction
  rather than angle. To support one, wire the spin callbacks (`setSpinDirection` /
  `getSpinDirection`) and add a `servo_type` option to switch the meaning of a position
  command. The interface already reports `has_spin` from those callbacks.
- **A different board entirely (for example an ESP32 with custom firmware).** Do not modify
  this driver. Copy the retained `svx_servo_generic_*` stub to a new `svx_servo_<board>_*`
  set and implement its callbacks. The SVX interface, the RUI, and `nepi_app_servo_auto` do
  not change, because they only ever speak degrees through `SVXActuatorIF`.
