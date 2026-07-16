#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi engine (nepi_engine) repo
# (see https://github.com/nepi-engine/nepi_engine)
#
# License: NEPI Engine repo source-code and NEPI Images that use this source-code
# are licensed under the "Numurus Software License",
# which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
#
# Redistributions in source code must retain this top-level comment block.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com
#

# SVX (servo) device interface.
# One SVX instance = one servo. Coordinating multiple servos (e.g. a pan/tilt
# turret) is the application's job, not this interface's.
# Open loop: position is reported as the last commanded value; nothing is measured.
# The API speaks degrees. Conversion to servo pulse width is a driver concern.

import os
import time
import copy
import math

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_system

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64, Header


from nepi_interfaces.srv import DeviceInfoQuery, DeviceInfoQueryResponse, DeviceInfoQueryRequest

from nepi_interfaces.msg import DeviceSVXStatus, ServoLimits
from nepi_interfaces.srv import SVXCapabilitiesQuery, SVXCapabilitiesQueryRequest, SVXCapabilitiesQueryResponse



from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF
from nepi_api.system_if import SettingsIF




class SVXActuatorIF:
    MAX_STATUS_UPDATE_RATE = 3
    MIN_LIMIT_ANGLE = 10
    # Backup Factory Control Values
    FACTORY_CONTROLS_DICT = {
                'reverse_enabled' : False,
                'speed_ratio' : 0.5,
                'spin_direction' : 1
    }

    ready = False

    node_if = None
    settings_if = None
    save_data_if = None

    status_msg = DeviceSVXStatus()
    info_report = DeviceInfoQueryResponse()
    caps_report = SVXCapabilitiesQueryResponse()

    has_absolute_positioning = False
    has_adjustable_speed = False
    has_limit_controls = False
    has_homing = False
    has_set_home = False
    has_spin = False
    has_goto_control = False
    has_stop_control = False


    move_decimal_place = 0
    report_decimal_place = 2


    # Define some member variables
    position_now_deg = 0.0
    position_goal_deg = 0.0
    home_pos_deg = 0.0

    min_hardstop_deg = 0.0
    max_hardstop_deg = 0.0
    min_softstop_deg = 0.0
    max_softstop_deg = 0.0

    reverse_enabled = False
    ri = 1

    spin_direction = 1
    is_spinning = False

    speed_ratio = 0.5
    speed_max_dps = 10


    data_source_description = 'servo'
    data_ref_description = 'servo_axis'


    ### IF Initialization
    def __init__(self,  device_info,
                 capSettings, factorySettings,
                 settingUpdateFunction, getSettingsFunction,
                 factoryControls , # Dictionary to be supplied by parent, specific key set is required
                 factoryLimits = None,
                 data_source_description = 'servo',
                 data_ref_description = 'servo_axis',
                 stopMovingCb = None, # Required; no args
                 gotoPositionCb = None, # None ==> No absolute positioning capability (position_deg)
                 getPositionCb = None, # Returns last sent position
                 setSoftLimitsCb = None, # Software defined limits
                 getSoftLimitsCb = None,
                 setSpinDirection = None,
                 getSpinDirection = None,
                 setSpeedRatioCb = None, # None ==> No speed adjustment capability; speed ratio arg
                 getSpeedRatioCb = None, # None ==> No speed adjustment capability; returns speed ratio
                 setSpeedMaxCb = None,
                 getSpeedMaxCb = None,
                 goHomeCb = None, # None ==> No homing; can still home if absolute positioning is supported
                 setHomePositionCb = None,
                 setHomePositionHereCb = None,
                 deviceResetCb = None,
                 log_name = None,
                 log_name_list = [],
                 msg_if = None
                ):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()
        self.namespace = nepi_sdk.create_namespace(self.node_namespace,'svx')
        self.data_products_list = []


        ##############################
        # Create Msg Class
        if msg_if is not None:
            self.msg_if = msg_if
        else:
            self.msg_if = MsgIF()
        self.log_name_list = copy.deepcopy(log_name_list)
        self.log_name_list.append(self.class_name)
        if log_name is not None:
            self.log_name_list.append(log_name)
        self.msg_if.pub_info("Starting SVX Device IF Initialization Processes", log_name_list = self.log_name_list)



        ##############################
        # Initialize Class Variables
        self.device_name = device_info["device_name"]
        self.path = device_info["path"]
        self.serial_num = device_info["serial_number"]
        self.hw_version = device_info["hw_version"]
        self.sw_version = device_info["sw_version"]

        self.status_msg.device_name = self.device_name
        self.status_msg.device_path = self.path
        self.status_msg.device_node_name = self.node_name
        self.status_msg.serial_num = self.serial_num
        self.status_msg.hw_version = self.hw_version
        self.status_msg.sw_version = self.sw_version

        self.info_report.device_name = self.device_name
        self.info_report.device_path = self.path
        self.info_report.node_name = self.node_name
        self.info_report.node_namespace = self.node_namespace
        self.info_report.serial_num = self.serial_num
        self.info_report.hw_version = self.hw_version
        self.info_report.sw_version = self.sw_version
        self.info_report.type = 'SVX'

        self.caps_report.device_name = self.device_name
        self.caps_report.device_path = self.path
        self.caps_report.device_node_name = self.node_name

        self.data_source_description = data_source_description
        self.data_ref_description = data_ref_description

        # Create and update factory controls dictionary
        self.factory_controls_dict = self.FACTORY_CONTROLS_DICT
        if factoryControls is not None:
            controls = list(factoryControls.keys())
            for control in controls:
                if self.factory_controls_dict.get(control) != None and factoryControls.get(control) != None:
                    self.factory_controls_dict[control] = factoryControls[control]

        self.deviceResetCb = deviceResetCb


        # Configure SVX Capabilities

        # STOP MOVE #############
        self.stopMovingCb = stopMovingCb
        if self.stopMovingCb is not None:
            self.has_stop_control = True

        # GET POSITION #############
        # Open loop: getPositionCb returns the last commanded position, nothing measured
        self.getPositionCb = getPositionCb

        # Soft Limits are handled by SVX IF at top level,
        # these are for updating the hardware if required
        self.setSoftLimitsCb = setSoftLimitsCb
        self.getSoftLimitsCb = getSoftLimitsCb
        if self.getSoftLimitsCb is not None:
            self.has_limit_controls = True

        # POSITION MOVE ############
        self.gotoPositionCb = gotoPositionCb
        if self.gotoPositionCb is not None:
            self.has_absolute_positioning = True
            self.has_goto_control = True

        # SPIN DIRECTION  #############
        self.setSpinDirection = setSpinDirection
        self.getSpinDirection = getSpinDirection
        if self.setSpinDirection is not None:
            self.has_spin = True

        # SPEED SETTINGS  #############
        self.setSpeedMaxCb = setSpeedMaxCb
        self.getSpeedMaxCb = getSpeedMaxCb
        if self.getSpeedMaxCb is not None:
            self.speed_max_dps = self.getSpeedMaxCb()

        self.setSpeedRatioCb = setSpeedRatioCb
        self.getSpeedRatioCb = getSpeedRatioCb
        if setSpeedRatioCb is not None and getSpeedRatioCb is not None:
            self.has_adjustable_speed = True

        # Homing  #############
        self.goHomeCb = goHomeCb
        self.setHomePositionCb = setHomePositionCb
        self.setHomePositionHereCb = setHomePositionHereCb
        if self.goHomeCb is not None:
            self.has_homing = True
        if self.setHomePositionHereCb is not None:
            self.has_set_home = True


        # Create Capabilities Report
        self.caps_report.mode = 0
        self.caps_report.has_absolute_positioning = self.has_absolute_positioning
        self.caps_report.has_adjustable_speed = self.has_adjustable_speed
        self.caps_report.has_limit_control = self.has_limit_controls
        self.caps_report.has_homing = self.has_homing
        self.caps_report.has_set_home = self.has_set_home
        self.caps_report.has_spin_control = self.has_spin
        self.caps_report.has_goto_control = self.has_goto_control
        self.caps_report.has_stop_control = self.has_stop_control
        self.caps_report.max_speed_dps = self.speed_max_dps


        #######################################
        # Set up factory limits

        if factoryLimits is None:
            factoryLimits = dict()
        self.factoryLimits = factoryLimits
        for key in ['min_hardstop_deg','max_hardstop_deg','min_softstop_deg','max_softstop_deg']:
            if key not in self.factoryLimits:
                self.factoryLimits[key] = 0

        self.min_hardstop_deg = self.factoryLimits['min_hardstop_deg']
        self.max_hardstop_deg = self.factoryLimits['max_hardstop_deg']
        self.min_softstop_deg = self.factoryLimits['min_softstop_deg']
        self.max_softstop_deg = self.factoryLimits['max_softstop_deg']


        ########################
        self.status_msg.data_source_description = self.data_source_description
        self.status_msg.data_ref_description = self.data_ref_description

        self.status_msg.has_absolute_positioning = self.has_absolute_positioning
        self.status_msg.has_adjustable_speed = self.has_adjustable_speed
        self.status_msg.has_limit_controls = self.has_limit_controls
        self.status_msg.has_homing = self.has_homing
        self.status_msg.has_set_home = self.has_set_home
        self.status_msg.has_spin = self.has_spin


        ##################################################
        ### Node Class Setup

        self.msg_if.pub_debug("Starting Node IF Initialization", log_name_list = self.log_name_list)
        alt_namespace = None
        if self.device_name != self.node_name:
            alt_namespace = self.node_namespace.replace(self.node_name,self.device_name)
        # Configs Config Dict ####################
        self.CONFIGS_DICT = {
                'init_callback': self.initCb,
                'reset_callback': self.resetCb,
                'factory_reset_callback': self.factoryResetCb,
                'init_configs': True,
                'namespace':  self.namespace,
                'alt_namespace': alt_namespace
        }


        # Params Config Dict ####################

        self.PARAMS_DICT = {
            'speed_max_dps': {
                'namespace': self.namespace,
                'factory_val': self.speed_max_dps
            },
            'speed_ratio': {
                'namespace': self.namespace,
                'factory_val': self.factory_controls_dict['speed_ratio']
            },
            'home_position/deg': {
                'namespace': self.namespace,
                'factory_val': 0.0
            },
            'reverse_enabled': {
                'namespace': self.namespace,
                'factory_val': self.factory_controls_dict['reverse_enabled']
            },
            'spin_direction': {
                'namespace': self.namespace,
                'factory_val': self.factory_controls_dict['spin_direction']
            },
            'max_softstop_deg': {
                'namespace': self.namespace,
                'factory_val': self.factoryLimits['max_softstop_deg']
            },
            'min_softstop_deg': {
                'namespace': self.namespace,
                'factory_val': self.factoryLimits['min_softstop_deg']
            },
            'move_decimal_place': {
                'namespace': self.namespace,
                'factory_val': 0
            },
            'report_decimal_place': {
                'namespace': self.namespace,
                'factory_val': 2
            }
        }


        # Services Config Dict ####################

        self.SRVS_DICT = {
            'device_info_query': {
                'namespace': self.namespace,
                'topic': 'device_info_query',
                'srv': DeviceInfoQuery,
                'req': DeviceInfoQueryRequest(),
                'resp': DeviceInfoQueryResponse(),
                'callback': self.info_query_callback
            },
            'capabilities_query': {
                'namespace': self.namespace,
                'topic': 'capabilities_query',
                'srv': SVXCapabilitiesQuery,
                'req': SVXCapabilitiesQueryRequest(),
                'resp': SVXCapabilitiesQueryResponse(),
                'callback': self.capabilities_query_callback
            }
        }


        self.PUBS_DICT = {
            'status_pub': {
                'namespace': self.namespace,
                'topic': 'status',
                'msg': DeviceSVXStatus,
                'qsize': 10,
                'latch': False
            }
        }



        # Subscribers Config Dict ####################
        # Exactly the Figure 5 control topics. On each control the IF clamps the
        # commanded value against the current soft limits, then calls the matching
        # callback. If a callback is None the corresponding has_* flag is False and
        # the control is a no-op.
        self.SUBS_DICT = {
            'goto_position': {
                'namespace': self.namespace,
                'topic': 'goto_position',
                'msg': Float32,
                'qsize': 1,
                'callback': self._gotoPositionCb,
                'callback_args': ()
            },
            'goto_ratio': {
                'namespace': self.namespace,
                'topic': 'goto_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self._gotoRatioCb,
                'callback_args': ()
            },
            'set_soft_limits': {
                'namespace': self.namespace,
                'topic': 'set_soft_limits',
                'msg': ServoLimits,
                'qsize': 1,
                'callback': self._setSoftLimitsCb,
                'callback_args': ()
            },
            'set_speed_ratio': {
                'namespace': self.namespace,
                'topic': 'set_speed_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self._setSpeedRatioCb,
                'callback_args': ()
            },
            'set_speed_max_dps': {
                'namespace': self.namespace,
                'topic': 'set_speed_max_dps',
                'msg': Float32,
                'qsize': 1,
                'callback': self._setMaxSpeedCb,
                'callback_args': ()
            },
            'set_reverse_enable': {
                'namespace': self.namespace,
                'topic': 'set_reverse_enable',
                'msg': Bool,
                'qsize': 1,
                'callback': self._setReverseEnableCb,
                'callback_args': ()
            },
            'stop_moving': {
                'namespace': self.namespace,
                'topic': 'stop_moving',
                'msg': Empty,
                'qsize': 1,
                'callback': self._stopMovingCb,
                'callback_args': ()
            },
            'go_home': {
                'namespace': self.namespace,
                'topic': 'go_home',
                'msg': Empty,
                'qsize': 1,
                'callback': self._goHomeCb,
                'callback_args': ()
            },
            'set_home_position': {
                'namespace': self.namespace,
                'topic': 'set_home_position',
                'msg': Float32,
                'qsize': 1,
                'callback': self._setHomePositionCb,
                'callback_args': ()
            },
            'set_home_position_here': {
                'namespace': self.namespace,
                'topic': 'set_home_position_here',
                'msg': Empty,
                'qsize': 1,
                'callback': self._setHomePositionHereCb,
                'callback_args': ()
            },
            'reset_device': {
                'namespace': self.namespace,
                'topic': 'reset_device',
                'msg': Empty,
                'qsize': 1,
                'callback': self._resetDeviceCb,
                'callback_args': ()
            },
            'set_spin_direction': {
                'namespace': self.namespace,
                'topic': 'set_spin_direction',
                'msg': Int32,
                'qsize': 1,
                'callback': self._setSpinDirectionCb,
                'callback_args': ()
            }
        }




        # Create Node Class ####################
        self.node_if = NodeClassIF(
                        configs_dict = self.CONFIGS_DICT,
                        params_dict = self.PARAMS_DICT,
                        services_dict = self.SRVS_DICT,
                        pubs_dict = self.PUBS_DICT,
                        subs_dict = self.SUBS_DICT,
                        log_name_list = self.log_name_list,
                            msg_if = self.msg_if
                        )

        success = nepi_sdk.wait()

        ##############################
        # Update vals from param server
        self.initCb(do_updates = True)
        self.publish_status()

        ##############################
        # Start Node Processes

        # Periodic status publishing
        status_update_rate = self.MAX_STATUS_UPDATE_RATE
        if status_update_rate < 1:
            status_update_rate = 1
        if status_update_rate > 10:
            status_update_rate = 10
        self.status_update_rate = status_update_rate
        self.msg_if.pub_info("Starting svx status publisher at hz: " + str(self.status_update_rate))
        status_pub_delay = float(1.0) / self.status_update_rate
        nepi_sdk.start_timer_process(status_pub_delay, self._publishStatusCb)

        ##############################
        # Start Additional System Processes

        ################################
        # Setup Settings IF Class ####################
        self.msg_if.pub_info("Starting Settings IF Initialization", log_name_list = self.log_name_list)
        settings_ns = self.namespace

        self.SETTINGS_DICT = {
                    'capSettings': capSettings,
                    'factorySettings': factorySettings,
                    'setSettingFunction': settingUpdateFunction,
                    'getSettingsFunction': getSettingsFunction

        }
        self.settings_if = SettingsIF(namespace = settings_ns,
                        settings_dict = self.SETTINGS_DICT,
                        log_name_list = self.log_name_list,
                            msg_if = self.msg_if
                        )
        #####################
        # Update Status Message
        nepi_sdk.sleep(1)
        if self.settings_if is not None:
            self.status_msg.settings_topic = self.settings_if.get_namespace()
            self.msg_if.pub_info("Using settings namespace: " + str(self.status_msg.settings_topic))

        ####################################
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete", log_name_list = self.log_name_list)
        ####################################



    def initCb(self, do_updates = False):
        if do_updates == True and self.node_if is not None:
            # This one comes from the parent
            if self.getSoftLimitsCb is not None:
                    [min_deg,max_deg] = self.getSoftLimitsCb()
                    if min_deg != -999:
                        self.min_softstop_deg = min_deg
                        self.node_if.set_param('min_softstop_deg', self.min_softstop_deg)
                    if max_deg != -999:
                        self.max_softstop_deg = max_deg
                        self.node_if.set_param('max_softstop_deg', self.max_softstop_deg)

        if self.node_if is not None:
            self.min_softstop_deg = self.node_if.get_param('min_softstop_deg')
            self.max_softstop_deg = self.node_if.get_param('max_softstop_deg')

            if None not in [self.min_softstop_deg,self.max_softstop_deg]:
                if self.setSoftLimitsCb is not None:
                    self.setSoftLimitsCb(self.min_softstop_deg, self.max_softstop_deg)

            if self.getSpeedMaxCb is None:
                self.speed_max_dps = self.node_if.get_param('speed_max_dps')
            if self.setSpeedMaxCb is not None:
                self.setSpeedMaxCb(self.speed_max_dps)

            speed_ratio = self.node_if.get_param('speed_ratio')
            if speed_ratio is not None:
                self.speed_ratio = speed_ratio
                if self.setSpeedRatioCb is not None:
                    self.setSpeedRatioCb(self.speed_ratio)

            self.home_pos_deg = self.node_if.get_param('home_position/deg')

            self.reverse_enabled = self.node_if.get_param('reverse_enabled')
            self.ri = -1 if self.reverse_enabled else 1

            self.spin_direction = self.node_if.get_param('spin_direction')
            if self.spin_direction is not None and self.setSpinDirection is not None:
                self.setSpinDirection(self.spin_direction)

            move_decimal_place = self.node_if.get_param('move_decimal_place')
            if move_decimal_place is not None:
                self.move_decimal_place = move_decimal_place
            report_decimal_place = self.node_if.get_param('report_decimal_place')
            if report_decimal_place is not None:
                self.report_decimal_place = report_decimal_place

        if do_updates == True:
            pass
        self.publish_status()

    def resetCb(self,do_updates = True):
        if self.node_if is not None:
            pass # self.node_if.reset_params()
        if self.save_data_if is not None:
            self.save_data_if.reset()
        if self.settings_if is not None:
            self.settings_if.reset()
        if do_updates == True:
            pass
        self.initCb(do_updates = True)

    def factoryResetCb(self,do_updates = True):
        if self.node_if is not None:
            pass # self.node_if.factory_reset_params()
        if self.save_data_if is not None:
            self.save_data_if.factory_reset()
        if self.settings_if is not None:
            self.settings_if.factory_reset()
        if do_updates == True:
            pass
        self.initCb(do_updates = True)



    ###############################
    # Class Methods

    def getPosAdj(self,deg):
        if deg is None:
            deg = 0.0
        return deg * self.ri

    def getPosRatioAdj(self,ratio):
        if self.reverse_enabled == True:
            ratio = 1 - ratio
        return ratio

    def getMinMaxAdj(self,min_deg,max_deg):
        if self.reverse_enabled == True:
            adj_min = copy.deepcopy(max_deg) * -1
            adj_max = copy.deepcopy(min_deg) * -1
        else:
            adj_min = min_deg
            adj_max = max_deg
        return adj_min,adj_max

    def getLimitsAdj(self,min_deg,max_deg):
        [adj_min,adj_max] = self.getMinMaxAdj(min_deg,max_deg)
        return adj_min,adj_max

    def getLimitsSoftstopAdj(self):
        [adj_min,adj_max] = self.getMinMaxAdj(self.min_softstop_deg,self.max_softstop_deg)
        return adj_min,adj_max


    def check_ready(self):
        """Returns the current ready state of the SVX device interface.

        Returns:
            bool: True if the interface has completed initialization, False otherwise.
        """
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        """Blocks until the SVX device interface is ready or the timeout expires.

        Args:
            timeout (float, optional): Maximum number of seconds to wait. Defaults to
                float('inf'), which waits indefinitely.

        Returns:
            bool: True if the device became ready within the timeout, False if it did not.
        """
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for connection", log_name_list = self.log_name_list)
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect", log_name_list = self.log_name_list)
            else:
                self.msg_if.pub_info("Connected", log_name_list = self.log_name_list)
        return self.ready



    def degToRatio(self, deg):
        span = (self.max_softstop_deg - self.min_softstop_deg)
        if span == 0:
            return 0.5
        if self.reverse_enabled == False:
            ratio = 1 - (deg - self.min_softstop_deg) / span
        else:
            ratio = (deg - self.min_softstop_deg) / span
        return ratio

    def ratioToDeg(self, ratio):
        span = (self.max_softstop_deg - self.min_softstop_deg)
        if self.reverse_enabled == False:
            deg = self.min_softstop_deg + (1-ratio) * span
        else:
            deg = self.max_softstop_deg - (1-ratio) * span
        return deg


    def positionWithinSoftLimits(self, deg):
        valid = False
        [adj_min,adj_max] = self.getLimitsSoftstopAdj()
        if (deg <= adj_max) and (deg >= adj_min):
            valid = True
        return valid


    def getPositionConditioned(self, deg):
        # Clamp the commanded value against the current soft limits (the trust
        # boundary: a bad goto is clamped here, never passed raw to hardware).
        [adj_min,adj_max] = self.getLimitsSoftstopAdj()
        if (deg > adj_max):
            deg = adj_max
        if (deg < adj_min):
            deg = adj_min

        place = self.move_decimal_place
        deg = nepi_utils.get_sign(deg) * (math.floor( abs(deg * 10**(place)) ) / 10**(place))
        return deg



    def _setSoftLimitsCb(self, msg):
        min_deg = msg.min_deg
        max_deg = msg.max_deg

        [min_adj,max_adj] = self.getLimitsAdj(min_deg,max_deg)

        valid = False
        if min_adj >= self.min_hardstop_deg and max_adj <= self.max_hardstop_deg and min_adj < max_adj:
            if abs(max_adj - min_adj) >= self.MIN_LIMIT_ANGLE:
                valid = True
                self.min_softstop_deg = min_adj
                self.max_softstop_deg = max_adj
                if self.setSoftLimitsCb is not None:
                    self.setSoftLimitsCb(min_adj,max_adj)
                if self.node_if is not None:
                    self.node_if.set_param('max_softstop_deg', max_adj)
                    self.node_if.set_param('min_softstop_deg', min_adj)

        if valid == False:
            self.msg_if.pub_warn("Invalid softstop requested " + str(msg))
        self.publish_status()


    def _stopMovingCb(self, _):
        self.msg_if.pub_warn("Got Stop Moving msg")
        self.stopServo()

    def stopServo(self):
        self.msg_if.pub_info("Stopping motion by request", log_name_list = self.log_name_list)
        if self.stopMovingCb is not None:
            self.stopMovingCb()
        elif self.gotoPositionCb is not None:
            stop_deg = self.status_msg.position_now_deg * self.ri
            self.gotoPositionCb(stop_deg)
        self.publish_status()


    def _setMaxSpeedCb(self, msg):
        max_speed = msg.data
        if (max_speed > 0.0):
            if self.setSpeedMaxCb is not None:
                self.setSpeedMaxCb(max_speed)
                self.speed_max_dps = max_speed
            if self.node_if is not None:
                self.node_if.set_param('speed_max_dps',max_speed)
            self.msg_if.pub_info("Updated max speed to " + str(max_speed))
            self.publish_status()

    def _setSpeedRatioCb(self, msg):
        ratio = nepi_utils.check_ratio(msg.data)
        self._setSpeedRatio(ratio)

    def _setSpeedRatio(self,speed_ratio):
        if self.has_adjustable_speed == True:
            self.speed_ratio = speed_ratio
            self.setSpeedRatioCb(speed_ratio)
            if self.node_if is not None:
                self.node_if.set_param('speed_ratio',speed_ratio)
            self.publish_status()


    def _setHomePositionCb(self, msg):
        deg_adj = self.getPosAdj(msg.data)
        if not self.positionWithinSoftLimits(deg_adj):
            self.msg_if.pub_warn("Requested home position is invalid... ignoring", log_name_list = self.log_name_list)
            return
        self.home_pos_deg = deg_adj
        self.msg_if.pub_info("Updated home position to " + "%.2f" % self.home_pos_deg)
        if self.node_if is not None:
            self.node_if.set_param('home_position/deg', deg_adj)
        if self.setHomePositionCb is not None:
            self.setHomePositionCb(self.home_pos_deg)
        self.publish_status()


    def _goHomeCb(self, _):
        self.msg_if.pub_warn("Got Go Home msg")
        self.goHome()

    def goHome(self):
        if self.gotoPositionCb is not None:
            self.position_goal_deg = self.home_pos_deg
            self.gotoPositionCb(self.position_goal_deg)
            self.publish_status()
        elif self.goHomeCb is not None:
            self.goHomeCb()


    def _gotoPositionCb(self, msg):
        deg_adj = self.getPosAdj(msg.data)
        self.gotoPosition(deg_adj)

    def gotoPosition(self,deg):
        if self.gotoPositionCb is None:
            return
        deg = self.getPositionConditioned(deg)
        self.position_goal_deg = deg
        self.gotoPositionCb(deg)
        self.publish_status()


    def _gotoRatioCb(self, msg):
        ratio = msg.data
        if (ratio < 0.0 or ratio > 1.0):
            self.msg_if.pub_warn("Invalid position ratio " + "%.2f" % ratio)
            return
        if self.gotoPositionCb is None:
            return
        self.position_goal_deg = self.ratioToDeg(ratio) # Function takes care of reverse conversion
        self.position_goal_deg = self.getPositionConditioned(self.position_goal_deg)
        self.gotoPositionCb(self.position_goal_deg)
        self.publish_status()


    def _setReverseEnableCb(self, msg):
        self.reverse_enabled = msg.data
        self.ri = -1 if msg.data == True else 1
        if self.node_if is not None:
            self.node_if.set_param('reverse_enabled', self.reverse_enabled)
        self.msg_if.pub_info("Set servo control to reverse=" + str(self.reverse_enabled))
        self.publish_status()


    def _setSpinDirectionCb(self, msg):
        if self.setSpinDirection is None:
            return
        spin_direction = 1 if msg.data >= 0 else -1
        self.spin_direction = spin_direction
        self.setSpinDirection(spin_direction)
        if self.node_if is not None:
            self.node_if.set_param('spin_direction', spin_direction)
        self.msg_if.pub_info("Set spin direction to " + str(spin_direction))
        self.publish_status()


    def _setHomePositionHereCb(self, _):
        self.home_pos_deg = self.status_msg.position_now_deg * self.ri
        if self.node_if is not None:
            self.node_if.set_param('home_position/deg', self.home_pos_deg)
        self.msg_if.pub_info("Home position set to: " + "%.2f" % self.home_pos_deg)
        if self.setHomePositionHereCb is not None:
            self.setHomePositionHereCb()
        self.publish_status()


    def _resetDeviceCb(self, _):
        self.msg_if.pub_warn("Got Reset Device msg")
        if self.deviceResetCb is not None:
            self.deviceResetCb()
        self.initCb(do_updates = True)


    ### Info callback
    def info_query_callback(self, _):
        """Service handler that returns the device info report.

        Returns:
            DeviceInfoQueryResponse: Response containing device name, path, node name,
                namespace, serial number, hardware version, software version, and type.
        """
        return self.info_report


    def capabilities_query_callback(self, _):
        """Service handler that returns the SVX capabilities report.

        Returns:
            SVXCapabilitiesQueryResponse: Response describing which SVX capabilities
                are supported, including absolute positioning, adjustable speed,
                limit control, homing, set-home, spin control, goto control, stop
                control, and the maximum speed in degrees per second.
        """
        return self.caps_report



    def _publishStatusCb(self,timer):
        self.publish_status()


    def publish_status(self, do_updates = False):
        """Builds and publishes the DeviceSVXStatus message to the status topic.

        Reports position open loop: the current position is taken from the driver's
        last-commanded value (getPositionCb) when available, otherwise from the last
        goal held by this interface. Recalculates ratio and degree fields for current
        and goal positions against soft-stop limits, then publishes the assembled
        DeviceSVXStatus message.

        Args:
            do_updates (bool, optional): Reserved for future use. Defaults to False.

        Returns:
            float: Elapsed time in seconds spent building and publishing the status.
        """
        start_time = nepi_utils.get_time()

        self.status_msg.move_decimal_place = self.move_decimal_place
        self.status_msg.report_decimal_place = self.report_decimal_place

        # Open loop: last commanded value is the position. getPositionCb returns
        # the last sent position; if not provided, fall back to our held goal.
        if self.getPositionCb is not None:
            deg = self.getPositionCb()
        else:
            deg = self.position_goal_deg * self.ri
        place = self.report_decimal_place
        deg = nepi_utils.get_sign(deg) * (math.floor( abs(deg * 10**(place)) ) / 10**(place))
        self.position_now_deg = deg

        now_deg_adj = self.getPosAdj(deg)
        self.status_msg.position_now_deg = now_deg_adj

        goal_deg_adj = self.getPosAdj(self.position_goal_deg)
        self.status_msg.position_goal_deg = goal_deg_adj
        self.status_msg.home_pos_deg = self.getPosAdj(self.home_pos_deg)

        [adj_ss_min,adj_ss_max] = self.getLimitsSoftstopAdj()
        self.status_msg.min_softstop_deg = adj_ss_min
        self.status_msg.max_softstop_deg = adj_ss_max

        span = (adj_ss_max - adj_ss_min)
        if span != 0:
            self.status_msg.position_now_ratio = 1 - (now_deg_adj - adj_ss_min) / span
            self.status_msg.position_goal_ratio = 1 - (goal_deg_adj - adj_ss_min) / span
        else:
            self.status_msg.position_now_ratio = 0.5
            self.status_msg.position_goal_ratio = 0.5

        self.status_msg.reverse_enabled = self.reverse_enabled

        if self.getSpinDirection is not None:
            self.spin_direction = self.getSpinDirection()
        self.status_msg.spin_direction = self.spin_direction
        self.status_msg.is_spinning = self.is_spinning

        if self.getSpeedRatioCb is not None:
            self.speed_ratio = self.getSpeedRatioCb()
        self.status_msg.speed_max_dps = self.speed_max_dps
        self.status_msg.speed_ratio = self.speed_ratio


        if self.node_if is not None:
            self.node_if.publish_pub('status_pub',self.status_msg)

        pub_time = nepi_utils.get_time() - start_time
        return pub_time
