#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi applications (nepi_drivers) repo
# (see https://https://github.com/nepi-engine/nepi_drivers)
#
# License: nepi applications are licensed under the "Numurus Software License",
# which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
#
# Redistributions in source code must retain this top-level comment bstab.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com

# SVX (servo) node for the Pololu Micro Maestro USB servo controller.
#
# Structure mirrors svx_servo_generic_node.py (the SVX category template, itself
# derived from the IQR PTX node); the serial send/receive machinery follows
# ptx_sidus_ss109_serial_node.py. The stub callbacks in the generic node are
# filled in here with the real Maestro protocol.
#
# One SVX device = one servo = one Maestro channel. A pan/tilt turret is two of
# these nodes (e.g. pan on channel 0, tilt on channel 1) coordinated by
# nepi_app_servo_auto. Multiple channel nodes can share one Maestro's single USB
# serial port safely: every serial transaction is wrapped in an advisory file
# lock (fcntl.flock) on the port, so two channel nodes never interleave bytes.
#
# The board is a plug-and-play USB device running Pololu factory firmware, so the
# whole hardware layer here is USB serial. The Maestro command set is documented
# at https://www.pololu.com/docs/0J40/5.e . This driver speaks the "Compact"
# protocol by default (works for a single controller regardless of its device
# number) and can speak the "Pololu" protocol (0xAA + device number) for
# daisy-chained boards, selected via the driver options.
#
# The SVX API speaks degrees. Converting a degree command to a servo pulse width
# (Maestro targets are in quarter-microseconds) is done here in the driver. The
# SVX IF handles the reverse-axis conversion, soft-limit clamping, and ratio math
# before it calls these callbacks, so the driver keeps a single pure linear
# degree <-> pulse-width map with no reverse logic of its own.

import copy
import threading

import serial

try:
    import fcntl  # Linux advisory locking so channel nodes can share one port
    HAVE_FCNTL = True
except Exception:
    HAVE_FCNTL = False

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_settings

from nepi_api.device_if_svx import SVXActuatorIF
from nepi_api.messages_if import MsgIF

PKG_NAME = 'SVX_SERVO_MAESTRO' # Use in display menus
FILE_TYPE = 'NODE'


class SvxServoMaestroNode:

    #############################
    # Pololu Maestro serial command set (Compact-protocol command bytes). The
    # Pololu-protocol form of each command is the same byte with its MSB cleared
    # (byte & 0x7F), sent after 0xAA and the device number.
    CMD_SET_TARGET = 0x84
    CMD_SET_SPEED = 0x87
    CMD_SET_ACCEL = 0x89
    CMD_GET_POSITION = 0x90
    CMD_GET_MOVING = 0x93
    CMD_GET_ERRORS = 0xA1
    CMD_GO_HOME = 0xA2

    # Maestro speed units are (0.25 us) / (10 ms) == 25 us/sec per unit.
    US_PER_SPEED_UNIT_PER_SEC = 25.0
    # Maestro targets/positions are in quarter-microseconds.
    QUS_PER_US = 4

    SERIAL_TIMEOUT_SEC = 0.25
    MAX_POSITION_UPDATE_RATE = 5

    #############################
    # Board-agnostic factory soft/hard limits (degrees) and the pulse-width
    # endpoints they map to. A common hobby positional servo sweeps +/-90 deg
    # over a 1000-2000 us pulse; the driver options override this per servo/horn.
    FACTORY_LIMITS_DICT = dict()
    FACTORY_LIMITS_DICT['min_hardstop_deg'] = -90
    FACTORY_LIMITS_DICT['max_hardstop_deg'] = 90
    FACTORY_LIMITS_DICT['min_softstop_deg'] = -90
    FACTORY_LIMITS_DICT['max_softstop_deg'] = 90

    limits_dict = copy.deepcopy(FACTORY_LIMITS_DICT)

    # Pulse-width mapping (microseconds). pulse_min_us <-> min_hardstop_deg and
    # pulse_max_us <-> max_hardstop_deg. Overridden from the driver options.
    pulse_min_us = 1000.0
    pulse_max_us = 2000.0
    accel_units = 0  # 0 == no acceleration limit

    CAP_SETTINGS = {
        'None' : {"type":"None","name":"None","options":[""]}
    }

    FACTORY_SETTINGS = {
        'None' : {"type":"None","name":"None","options":[""]}
    }

    FACTORY_SETTINGS_OVERRIDES = dict()

    settingFunctions = {
        'None' : {'get':'getNone', 'set': 'setNone'}
    }

    device_info_dict = dict(device_name = "",
                            path = "",
                            serial_number = "",
                            hw_version = "",
                            sw_version = "")

    # Initialize some parameters
    serial_num = "Unknown"
    hw_version = "Unknown"
    sw_version = "Unknown"
    svx_if = None

    # Board link
    serial_port = None
    baud_int = 9600
    channel = 0
    device_number = 12          # Maestro factory-default device number
    use_pololu_protocol = False # Compact protocol by default

    # Open loop from the servo's point of view, but the Maestro reports the pulse
    # width it is currently transmitting, so getPosition can reflect a slew.
    position_deg = 0.0
    goal_deg = 0.0
    home_pos_deg = 0.0
    speed_ratio = 0.5
    speed_max_dps = 20.0
    spin_direction = 1

    connected = False

    drv_dict = dict()


    ################################################
    DEFAULT_NODE_NAME = PKG_NAME.lower() + "_node"

    def __init__(self):
        ####  NODE Initialization ####
        nepi_sdk.init_node(name= self.DEFAULT_NODE_NAME)
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        ##############################
        # Create Msg Class
        self.msg_if = MsgIF(log_name = self.class_name)
        self.msg_if.pub_info("Starting Node Initialization Processes")

        # In-process guard around each serial transaction (paired with the
        # cross-process fcntl lock inside send_cmd).
        self.serial_lock = threading.Lock()

        ##############################
        # Initialize Class Variables

        # Get required drv driver dict info
        self.drv_dict = nepi_sdk.get_param('~drv_dict',dict())
        try:
            self.device_name = self.drv_dict['DEVICE_DICT']['device_name']
            self.device_path = self.drv_dict['DEVICE_DICT']['device_path']
            self.port_str = self.drv_dict['DEVICE_DICT']['device_path']

            self.channel = int(self.drv_dict['DEVICE_DICT'].get('channel', 0))
            self.baud_int = int(self.drv_dict['DEVICE_DICT'].get('baud_str', '9600'))
            self.device_number = int(self.drv_dict['DEVICE_DICT'].get('device_number', 12))
            self.serial_num = str(self.drv_dict['DEVICE_DICT'].get('serial_number', 'Unknown'))
        except Exception as e:
            self.msg_if.pub_warn("Failed to load Device Dict " + str(e))
            nepi_sdk.signal_shutdown(self.node_name + ": Shutting down because no valid Device Dict")
            return

        # Optional discovery OPTIONS (protocol + pulse-width + degree range + accel)
        try:
            options = self.drv_dict.get('DISCOVERY_DICT', {}).get('OPTIONS', {})
            protocol = options.get('protocol', {}).get('value', 'Compact')
            self.use_pololu_protocol = (str(protocol).lower() == 'pololu')

            self.pulse_min_us = float(options.get('pulse_min_us', {}).get('value', self.pulse_min_us))
            self.pulse_max_us = float(options.get('pulse_max_us', {}).get('value', self.pulse_max_us))
            self.accel_units = int(options.get('accel_units', {}).get('value', self.accel_units))

            min_deg = float(options.get('min_deg', {}).get('value', self.FACTORY_LIMITS_DICT['min_hardstop_deg']))
            max_deg = float(options.get('max_deg', {}).get('value', self.FACTORY_LIMITS_DICT['max_hardstop_deg']))
            self.FACTORY_LIMITS_DICT['min_hardstop_deg'] = min_deg
            self.FACTORY_LIMITS_DICT['max_hardstop_deg'] = max_deg
            self.FACTORY_LIMITS_DICT['min_softstop_deg'] = min_deg
            self.FACTORY_LIMITS_DICT['max_softstop_deg'] = max_deg
            self.limits_dict = copy.deepcopy(self.FACTORY_LIMITS_DICT)
        except Exception as e:
            self.msg_if.pub_warn("Failed to parse driver OPTIONS, using defaults: " + str(e))

        if self.pulse_max_us <= self.pulse_min_us:
            self.msg_if.pub_warn("Invalid pulse-width range, reverting to 1000-2000us")
            self.pulse_min_us = 1000.0
            self.pulse_max_us = 2000.0

        ################################################
        self.msg_if.pub_info("Connecting to Maestro on port " + str(self.port_str) +
                             " channel " + str(self.channel))
        self.connected = self.connect()
        if self.connected == False:
            self.msg_if.pub_info("Shutting down node, Unable to connect to Maestro servo controller")
            nepi_sdk.signal_shutdown("Unable to connect to servo device")
            return

        self.msg_if.pub_info("... Connected!")

        # Initialize settings
        self.cap_settings = self.getCapSettings()
        self.factory_settings = self.getFactorySettings()

        self.device_info_dict["device_name"] = self.device_name
        self.device_info_dict["path"] = self.device_path
        self.device_info_dict["serial_number"] = self.serial_num
        self.device_info_dict["hw_version"] = self.hw_version
        self.device_info_dict["sw_version"] = self.sw_version

        #Factory Control Values
        self.FACTORY_CONTROLS = {
            'reverse_enabled' : False,
            'speed_ratio' : 0.5,
            'spin_direction' : 1
        }

        # Push the initial acceleration + speed to the board before we come up.
        self.driver_setAcceleration(self.accel_units)
        self.driver_setSpeedRatio(self.speed_ratio)

        # Launch the SVX interface -- it subscribes/advertises all the servo
        # control topics, publishes status, and serves capabilities. We pass the
        # Figure 1 callbacks; the interface derives each has_* capability flag
        # from which callbacks are non-None. A positional (non-continuous) servo
        # has no spin, so the spin callbacks are None and has_spin reports False.
        self.msg_if.pub_info("Launching NEPI SVX interface...")
        self.svx_if = SVXActuatorIF(device_info = self.device_info_dict,
                                    capSettings = self.cap_settings,
                                    factorySettings = self.factory_settings,
                                    settingUpdateFunction = self.settingUpdateFunction,
                                    getSettingsFunction = self.getSettings,
                                    factoryControls = self.FACTORY_CONTROLS,
                                    factoryLimits = self.FACTORY_LIMITS_DICT,
                                    stopMovingCb = self.stopMoving,
                                    gotoPositionCb = self.gotoPosition,
                                    getPositionCb = self.getPosition,
                                    setSoftLimitsCb = self.setSoftLimits,
                                    getSoftLimitsCb = self.getSoftLimits,
                                    setSpinDirection = None,
                                    getSpinDirection = None,
                                    setSpeedRatioCb = self.setSpeedRatio,
                                    getSpeedRatioCb = self.getSpeedRatio,
                                    setSpeedMaxCb = self.setSpeedMax,
                                    getSpeedMaxCb = self.getSpeedMax,
                                    goHomeCb = self.goHome,
                                    setHomePositionCb = self.setHomePosition,
                                    setHomePositionHereCb = self.setHomePositionHere,
                                    deviceResetCb = self.resetDevice
                                    )
        self.msg_if.pub_info(" ... SVX interface running")

        # Poll the Maestro for the pulse width it is currently transmitting so
        # the reported position tracks a speed/accel-limited slew.
        update_interval = float(1.0) / self.MAX_POSITION_UPDATE_RATE
        nepi_sdk.start_timer_process(update_interval, self.updatePositionHandler, oneshot = True)

        # Initialization Complete
        self.msg_if.pub_info("Initialization Complete")
        #Set up node shutdown
        nepi_sdk.on_shutdown(self.cleanup_actions)
        # Spin forever
        nepi_sdk.spin()


    def updatePositionHandler(self, timer):
        stime = nepi_utils.get_time()
        pos_deg = self.driver_getPosition()
        if pos_deg is not None:
            self.position_deg = pos_deg
        # Surface any accumulated board errors to the log/UI.
        self.driver_checkErrors()
        gtime = nepi_utils.get_time() - stime
        next_delay = max(0.05, float(1.0) / self.MAX_POSITION_UPDATE_RATE - gtime)
        nepi_sdk.start_timer_process(next_delay, self.updatePositionHandler, oneshot = True)


    #**********************
    # Device setting functions

    def getCapSettings(self):
        return self.CAP_SETTINGS

    def getFactorySettings(self):
        settings = self.getSettings()
        for setting_name in settings.keys():
            if setting_name in self.FACTORY_SETTINGS_OVERRIDES:
                settings[setting_name]['value'] = self.FACTORY_SETTINGS_OVERRIDES[setting_name]
        return settings

    def getSettings(self):
        settings = dict()
        for setting_name in self.cap_settings.keys():
            cap_setting = self.cap_settings[setting_name]
            setting = dict()
            setting["name"] = setting_name
            setting["type"] = cap_setting['type']
            val = None
            if setting_name in self.settingFunctions.keys():
                function_str_name = self.settingFunctions[setting_name]['get']
                get_function = globals()[function_str_name]
                val = get_function(self)
                if val is not None:
                    setting["value"] = str(val)
                    settings[setting_name] = setting
        return settings

    def setSetting(self, setting_name, val):
        success = False
        if setting_name in self.settingFunctions.keys():
            function_str_name = self.settingFunctions[setting_name]['set']
            set_function = getattr(self, function_str_name, None)
            if set_function is None:
                self.msg_if.pub_warn("Missing set function: " + function_str_name)
            else:
                success = set_function(val)
        return success

    def settingUpdateFunction(self, setting):
        success = False
        setting_str = str(setting)
        [setting_name, s_type, data] = nepi_settings.get_data_from_setting(setting)
        if data is not None:
            setting_data = data
            found_setting = False
            if setting_name in self.cap_settings.keys():
                found_setting = True
                success, msg = self.setSetting(setting_name,setting_data)
                if success:
                    msg = ( self.node_name  + " UPDATED SETTINGS " + setting_str)
            if found_setting is False:
                msg = (self.node_name  + " Setting name" + setting_str + " is not supported")
        else:
            msg = (self.node_name  + " Setting data" + setting_str + " is None")
        return success, msg


    ##############
    ### Settings Functions

    global getNone
    def getNone(self):
        val = '-999'
        return val

    global setNone
    def setNone(self,val):
        success = True
        return success


    #######################
    ### SVX IF Callbacks
    #
    # The SVX interface clamps any commanded position against the soft limits and
    # applies the reverse-axis conversion before calling gotoPosition, so the
    # driver receives a safe degree value in its own (non-reversed) frame.

    def stopMoving(self):
        # Positional servo: "stop" == hold where it is now. Read the pulse width
        # the Maestro is currently transmitting and re-command it as the target,
        # which halts any in-progress speed/accel-limited slew.
        pos_deg = self.driver_getPosition()
        if pos_deg is not None:
            self.driver_moveToPosition(pos_deg)

    def gotoPosition(self, position_deg):
        # position_deg is already soft-limit clamped and reverse-adjusted by the IF.
        self.goal_deg = position_deg
        self.driver_moveToPosition(position_deg)

    def getPosition(self):
        # Reports the pulse width the Maestro is transmitting (refreshed by the
        # poll timer); falls back to the last commanded value if a read fails.
        return self.position_deg

    def setSoftLimits(self, min_deg, max_deg):
        self.limits_dict['min_softstop_deg'] = min_deg
        self.limits_dict['max_softstop_deg'] = max_deg

    def getSoftLimits(self):
        return self.limits_dict['min_softstop_deg'], self.limits_dict['max_softstop_deg']

    def setSpeedRatio(self, ratio):
        self.speed_ratio = ratio
        self.driver_setSpeedRatio(ratio)

    def getSpeedRatio(self):
        return self.speed_ratio

    def setSpeedMax(self, speed_max_dps):
        self.speed_max_dps = speed_max_dps
        # Re-apply so the ratio still means the same fraction of the new max.
        self.driver_setSpeedRatio(self.speed_ratio)

    def getSpeedMax(self):
        return self.speed_max_dps

    def goHome(self):
        self.gotoPosition(self.home_pos_deg)

    def setHomePosition(self, position_deg):
        self.home_pos_deg = position_deg

    def setHomePositionHere(self):
        self.home_pos_deg = self.position_deg

    def resetDevice(self):
        # Send the board to its firmware home positions, clear errors, and
        # restore this node's factory config.
        self.driver_goHome()
        self.driver_checkErrors(clear_only = True)
        self.speed_ratio = 0.5
        self.speed_max_dps = 20.0
        self.limits_dict = copy.deepcopy(self.FACTORY_LIMITS_DICT)
        self.driver_setAcceleration(self.accel_units)
        self.driver_setSpeedRatio(self.speed_ratio)


    #######################
    ### Unit Conversions (degrees <-> microseconds <-> quarter-microseconds)
    #
    # Pure linear map; reverse-axis handling lives in the SVX IF, not here.

    def deg2us(self, deg):
        min_deg = self.FACTORY_LIMITS_DICT['min_hardstop_deg']
        max_deg = self.FACTORY_LIMITS_DICT['max_hardstop_deg']
        frac = (deg - min_deg) / (max_deg - min_deg)
        us = self.pulse_min_us + frac * (self.pulse_max_us - self.pulse_min_us)
        # Never let a conversion escape the servo's pulse-width endpoints.
        return max(self.pulse_min_us, min(self.pulse_max_us, us))

    def us2deg(self, us):
        min_deg = self.FACTORY_LIMITS_DICT['min_hardstop_deg']
        max_deg = self.FACTORY_LIMITS_DICT['max_hardstop_deg']
        frac = (us - self.pulse_min_us) / (self.pulse_max_us - self.pulse_min_us)
        return min_deg + frac * (max_deg - min_deg)

    def deg2target(self, deg):
        # Maestro target is in quarter-microseconds (rounded to an int).
        return int(round(self.deg2us(deg) * self.QUS_PER_US))

    def target2deg(self, target_qus):
        return self.us2deg(float(target_qus) / self.QUS_PER_US)

    def usPerDeg(self):
        min_deg = self.FACTORY_LIMITS_DICT['min_hardstop_deg']
        max_deg = self.FACTORY_LIMITS_DICT['max_hardstop_deg']
        span_deg = (max_deg - min_deg)
        if span_deg == 0:
            return 0.0
        return (self.pulse_max_us - self.pulse_min_us) / span_deg

    def ratio2speedUnits(self, ratio):
        # Maestro speed unit == 25 us/sec. Convert deg/sec to us/sec to units.
        dps = ratio * self.speed_max_dps
        us_per_sec = dps * self.usPerDeg()
        units = int(round(us_per_sec / self.US_PER_SPEED_UNIT_PER_SEC))
        # A Maestro speed of 0 means "unlimited", which is not what ratio 0.0
        # should do, so clamp to the slowest non-zero speed.
        if units < 1:
            units = 1
        return units


    #######################
    ### Driver Interface Functions (Maestro serial transactions)

    def driver_moveToPosition(self, deg):
        target = self.deg2target(deg)
        low = target & 0x7F
        high = (target >> 7) & 0x7F
        payload = bytes([self.channel & 0x7F, low, high])
        return self.send_cmd(self.CMD_SET_TARGET, payload)

    def driver_setSpeedRatio(self, ratio):
        units = self.ratio2speedUnits(ratio)
        low = units & 0x7F
        high = (units >> 7) & 0x7F
        payload = bytes([self.channel & 0x7F, low, high])
        return self.send_cmd(self.CMD_SET_SPEED, payload)

    def driver_setAcceleration(self, units):
        units = max(0, int(units))
        low = units & 0x7F
        high = (units >> 7) & 0x7F
        payload = bytes([self.channel & 0x7F, low, high])
        return self.send_cmd(self.CMD_SET_ACCEL, payload)

    def driver_getPosition(self):
        payload = bytes([self.channel & 0x7F])
        resp = self.send_cmd(self.CMD_GET_POSITION, payload, read_len = 2)
        if resp is None or len(resp) < 2:
            return None
        target_qus = resp[0] | (resp[1] << 8)
        if target_qus == 0:
            # 0 means the Maestro is not driving the channel (no pulse) -- no
            # meaningful position to report.
            return None
        return self.target2deg(target_qus)

    def driver_getMovingState(self):
        resp = self.send_cmd(self.CMD_GET_MOVING, b'', read_len = 1)
        if resp is None or len(resp) < 1:
            return None
        return resp[0] == 0x01

    def driver_goHome(self):
        return self.send_cmd(self.CMD_GO_HOME, b'')

    def driver_checkErrors(self, clear_only = False):
        # Get Errors returns two little-endian bytes of error flags and clears
        # them on the board. Surface any set flags for the operator.
        resp = self.send_cmd(self.CMD_GET_ERRORS, b'', read_len = 2)
        if resp is None or len(resp) < 2:
            return
        if clear_only:
            return
        error_bits = resp[0] | (resp[1] << 8)
        if error_bits != 0:
            self.msg_if.pub_warn("Maestro error flags: 0x{:04X}".format(error_bits))


    #######################
    ### Connection

    def connect(self):
        success = False
        try:
            self.msg_if.pub_info("Opening serial port " + str(self.port_str) +
                                 " with baudrate: " + str(self.baud_int))
            self.serial_port = serial.Serial(self.port_str, self.baud_int,
                                             timeout = self.SERIAL_TIMEOUT_SEC)
            self.msg_if.pub_info("Serial port opened")
        except Exception as e:
            self.msg_if.pub_warn("Failed to open serial port at: " + str(self.port_str) +
                                 " (" + str(e) + ")")
            return False

        # Handshake: read (and clear) the error register. A Maestro answers Get
        # Errors with exactly two bytes; anything else means this is not the
        # Command Port (or the Pololu device number is wrong).
        resp = self.send_cmd(self.CMD_GET_ERRORS, b'', read_len = 2)
        if resp is not None and len(resp) == 2:
            success = True
            self.hw_version = "Pololu Maestro"
            self.sw_version = "Factory Firmware"
        else:
            self.msg_if.pub_warn("No valid Maestro response on " + str(self.port_str) +
                                 "; is this the Maestro Command Port?")
            try:
                self.serial_port.close()
            except Exception:
                pass
            self.serial_port = None
        return success


    def send_cmd(self, cmd_byte, payload = b'', read_len = 0):
        # Frame a command in the configured protocol, send it, and (optionally)
        # read a fixed-length response. Wrapped in an in-process lock AND an
        # advisory file lock on the serial device so that multiple channel nodes
        # sharing one Maestro port never interleave their bytes.
        if self.serial_port is None:
            return None

        if self.use_pololu_protocol:
            frame = bytes([0xAA, self.device_number & 0x7F, cmd_byte & 0x7F]) + payload
        else:
            frame = bytes([cmd_byte]) + payload

        response = None
        with self.serial_lock:
            locked = False
            try:
                if HAVE_FCNTL:
                    fcntl.flock(self.serial_port.fileno(), fcntl.LOCK_EX)
                    locked = True
                self.serial_port.reset_input_buffer()
                self.serial_port.write(frame)
                self.serial_port.flush()
                if read_len > 0:
                    data = self.serial_port.read(read_len)
                    if data is not None and len(data) == read_len:
                        response = bytearray(data)
            except Exception as e:
                self.msg_if.pub_warn("Serial transaction failed: " + str(e))
                response = None
            finally:
                if locked:
                    try:
                        fcntl.flock(self.serial_port.fileno(), fcntl.LOCK_UN)
                    except Exception:
                        pass
        return response


    #######################
    ### Cleanup processes on node shutdown
    def cleanup_actions(self):
        self.msg_if.pub_info("Shutting down: Executing script cleanup actions")
        if self.serial_port is not None:
            try:
                # Leave the channel un-driven (target 0 == no pulse) so the servo
                # is not held under power after the node exits.
                self.send_cmd(self.CMD_SET_TARGET, bytes([self.channel & 0x7F, 0, 0]))
            except Exception:
                pass
            try:
                self.serial_port.close()
            except Exception:
                pass


if __name__ == '__main__':
    node = SvxServoMaestroNode()
