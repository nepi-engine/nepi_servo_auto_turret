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

# SVX (servo) generic node -- single-servo driver STUB.
#
# One SVX device = one servo. This node registers a single servo with the NEPI
# SVXActuatorIF using the Figure 1 callback set and provides board-agnostic
# callback stubs. It does NOT talk to any hardware.
#
# DEVELOPER TODO (the hardware layer is the next task):
#   The servo-controller board is not yet chosen (Pololu Maestro vs. ESP32). The
#   API speaks degrees; converting a degree command to a servo pulse width and
#   pushing it over the board link (USB serial for the Maestro, or custom ESP32
#   firmware) is a DRIVER concern and belongs in the callback bodies below (or a
#   companion svx_servo_generic_driver.py). Every hardware-facing callback below
#   is a clearly marked stub. Do NOT add serial/USB library imports until the
#   board is selected -- keep this interface board-agnostic (CODEX: hardware
#   abstraction first).

import copy

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_settings

from nepi_api.device_if_svx import SVXActuatorIF
from nepi_api.messages_if import MsgIF

PKG_NAME = 'SVX_SERVO_GENERIC' # Use in display menus
FILE_TYPE = 'NODE'


class SvxServoGenericNode:

    # Board-agnostic factory soft/hard limits (degrees). A hobby positional servo
    # is commonly +/- 90 deg of travel; adjust once the board and horn are known.
    FACTORY_LIMITS_DICT = dict()
    FACTORY_LIMITS_DICT['min_hardstop_deg'] = -90
    FACTORY_LIMITS_DICT['max_hardstop_deg'] = 90
    FACTORY_LIMITS_DICT['min_softstop_deg'] = -90
    FACTORY_LIMITS_DICT['max_softstop_deg'] = 90

    limits_dict = copy.deepcopy(FACTORY_LIMITS_DICT)

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

    # Open loop: we only know the last commanded position (nothing is measured).
    position_deg = 0.0
    home_pos_deg = 0.0
    speed_ratio = 0.5
    speed_max_dps = 20.0
    spin_direction = 1

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

        ##############################
        # Initialize Class Variables

        # Get required drv driver dict info
        self.drv_dict = nepi_sdk.get_param('~drv_dict',dict())
        try:
            self.device_name = self.drv_dict['DEVICE_DICT']['device_name']
            self.device_path = self.drv_dict['DEVICE_DICT']['device_path']
        except Exception as e:
            self.msg_if.pub_warn("Failed to load Device Dict " + str(e))
            nepi_sdk.signal_shutdown(self.node_name + ": Shutting down because no valid Device Dict")
            return

        ################################################
        # STUB: no hardware connection is attempted. Once the board is chosen,
        # open the board link here (USB serial for Maestro, or ESP32 firmware
        # link) and populate serial_num / hw_version / sw_version from it.
        self.connected = self.connect()
        if self.connected == False:
            self.msg_if.pub_info("Shutting down node, Unable to connect to servo device")
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

        # Launch the SVX interface -- takes care of subscribing/advertising all
        # the servo control topics, the status publisher, and the capabilities
        # service. We pass the Figure 1 callbacks; each hardware-facing one is a
        # stub below. Pass None for any capability this board will not support so
        # the corresponding has_* flag reports False and the control is a no-op.
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
                                    setSpinDirection = self.setSpinDirection,
                                    getSpinDirection = self.getSpinDirection,
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

        # Initialization Complete
        self.msg_if.pub_info("Initialization Complete")
        #Set up node shutdown
        nepi_sdk.on_shutdown(self.cleanup_actions)
        # Spin forever
        nepi_sdk.spin()


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

    def settingUpdateFunction(self,setting):
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
    ### SVX IF Callbacks -- HARDWARE STUBS
    #
    # Every callback below is a stub. The SVX interface has already clamped any
    # commanded position against the soft limits before calling gotoPosition, so
    # the driver receives a safe degree value. The remaining work is board I/O:
    #   deg -> pulse-width -> board link (Maestro USB serial or ESP32 firmware).

    def stopMoving(self):
        # TODO(board): command the servo to hold/stop. For a positional servo,
        # re-issue the current position; for a continuous servo, command zero speed.
        self.msg_if.pub_info("[stub] stopMoving")

    def gotoPosition(self, position_deg):
        # TODO(board): convert position_deg -> pulse width and send to the board.
        # position_deg is already soft-limit clamped by SVXActuatorIF.
        self.position_deg = position_deg
        self.msg_if.pub_info("[stub] gotoPosition deg=" + str(position_deg))

    def getPosition(self):
        # Open loop: return the last commanded position (nothing is measured).
        return self.position_deg

    def setSoftLimits(self, min_deg, max_deg):
        # TODO(board): push limits to the board if it enforces them in firmware.
        self.limits_dict['min_softstop_deg'] = min_deg
        self.limits_dict['max_softstop_deg'] = max_deg

    def getSoftLimits(self):
        return self.limits_dict['min_softstop_deg'], self.limits_dict['max_softstop_deg']

    def setSpinDirection(self, spin_direction):
        # TODO(board): only meaningful for a continuous-rotation servo.
        self.spin_direction = spin_direction

    def getSpinDirection(self):
        return self.spin_direction

    def setSpeedRatio(self, ratio):
        # TODO(board): map 0.0-1.0 ratio to the board's speed/slew-rate setting.
        self.speed_ratio = ratio

    def getSpeedRatio(self):
        return self.speed_ratio

    def setSpeedMax(self, speed_max_dps):
        # TODO(board): record/apply the deg/sec that a ratio of 1.0 corresponds to.
        self.speed_max_dps = speed_max_dps

    def getSpeedMax(self):
        return self.speed_max_dps

    def goHome(self):
        # TODO(board): drive to home. Left as goto if absolute positioning is used.
        self.gotoPosition(self.home_pos_deg)

    def setHomePosition(self, position_deg):
        self.home_pos_deg = position_deg

    def setHomePositionHere(self):
        self.home_pos_deg = self.position_deg

    def resetDevice(self):
        # TODO(board): restore board defaults if supported.
        self.speed_ratio = 0.5
        self.spin_direction = 1
        self.limits_dict = copy.deepcopy(self.FACTORY_LIMITS_DICT)


    #######################
    ### Connection STUB

    def connect(self):
        # STUB: no board link is opened. Returns True so the interface comes up
        # and the ROS surface is testable without hardware. Replace with a real
        # board handshake (Maestro USB serial or ESP32 firmware link) that also
        # populates self.serial_num / self.hw_version / self.sw_version.
        self.msg_if.pub_warn("[stub] connect(): no servo-controller board selected; "
                             "bringing up ROS interface only. Implement board I/O next.")
        return True


    #######################
    ### Cleanup processes on node shutdown
    def cleanup_actions(self):
        self.msg_if.pub_info("Shutting down: Executing script cleanup actions")
        # TODO(board): close the board link here once it exists.


if __name__ == '__main__':
    node = SvxServoGenericNode()
