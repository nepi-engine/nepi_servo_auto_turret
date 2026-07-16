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

# Connect client for the SVX (servo) device interface.
# One SVX instance = one servo. Mirrors connect_device_if_ptx.py, collapsed to a
# single axis: publishers for every Figure 5 control topic, a subscriber for
# DeviceSVXStatus, and a caller for SVXCapabilitiesQuery.

import os
import time
import copy
import math


from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64, Header
from nepi_interfaces.msg import SaveDataRate
from nepi_interfaces.msg import DeviceSVXStatus, ServoLimits
from nepi_interfaces.srv import SVXCapabilitiesQuery, SVXCapabilitiesQueryRequest, SVXCapabilitiesQueryResponse

from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF
from nepi_api.system_if import SaveDataIF, SettingsIF


from nepi_api.connect_node_if import ConnectNodeClassIF

#########################################
# Node Class
#########################################

class ConnectSVXDeviceIF:
    CONNECTED_TIMEOUT = 2
    msg_if = None
    ready = False
    namespace = '~'

    con_node_if = None

    status_msg = None
    connected = False
    last_status_time = 0
    position_deg = 0
    last_position_deg = 0
    moving = False

    statusCb = None

    speed_max_dps = 10
    #######################
    ### IF Initialization
    def __init__(self,
                namespace = None,
                statusCb = None,
                log_name = None,
                log_name_list = [],
                msg_if = None
                ):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        ##############################
        # Create Msg Class
        self.msg_if = MsgIF(log_name = self.class_name)
        self.msg_if.pub_info("Starting IF Initialization Processes")


        ##############################
        # Initialize Class Variables

        if namespace is None:
            return

        self.namespace = nepi_sdk.get_full_namespace(namespace)
        self.msg_if.pub_info("Using SVX Namespace: " + self.namespace)

        self.statusCb = statusCb


        ##############################
        ## Node Setup

        # Configs Config Dict ####################
        self.CONFIGS_DICT = {
                'namespace': self.namespace
        }


        # Services Config Dict ####################
        self.SRVS_DICT = {
            'capabilities_query': {
                'namespace': self.namespace,
                'topic': 'capabilities_query',
                'srv': SVXCapabilitiesQuery,
                'req': SVXCapabilitiesQueryRequest(),
                'resp': SVXCapabilitiesQueryResponse()
            }
        }


        # Publishers Config Dict ####################
        # One publisher per Figure 5 control topic.
        self.PUBS_DICT = {
            'goto_position': {
                'namespace': self.namespace,
                'topic': 'goto_position',
                'msg': Float32,
                'qsize': 1,
            },
            'goto_ratio': {
                'namespace': self.namespace,
                'topic': 'goto_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'set_soft_limits': {
                'namespace': self.namespace,
                'topic': 'set_soft_limits',
                'msg': ServoLimits,
                'qsize': 1,
            },
            'set_speed_ratio': {
                'namespace': self.namespace,
                'topic': 'set_speed_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'set_speed_max_dps': {
                'namespace': self.namespace,
                'topic': 'set_speed_max_dps',
                'msg': Float32,
                'qsize': 1,
            },
            'set_reverse_enable': {
                'namespace': self.namespace,
                'topic': 'set_reverse_enable',
                'msg': Bool,
                'qsize': 1,
            },
            'stop_moving': {
                'namespace': self.namespace,
                'topic': 'stop_moving',
                'msg': Empty,
                'qsize': 1,
            },
            'go_home': {
                'namespace': self.namespace,
                'topic': 'go_home',
                'msg': Empty,
                'qsize': 1,
            },
            'set_home_position': {
                'namespace': self.namespace,
                'topic': 'set_home_position',
                'msg': Float32,
                'qsize': 1,
            },
            'set_home_position_here': {
                'namespace': self.namespace,
                'topic': 'set_home_position_here',
                'msg': Empty,
                'qsize': 1,
            },
            'reset_device': {
                'namespace': self.namespace,
                'topic': 'reset_device',
                'msg': Empty,
                'qsize': 1,
            },
            'set_spin_direction': {
                'namespace': self.namespace,
                'topic': 'set_spin_direction',
                'msg': Int32,
                'qsize': 1,
            }
        }

        # Subscribers Config Dict ####################
        self.SUBS_DICT = {
            'status_sub': {
                'namespace': self.namespace,
                'topic': 'status',
                'msg': DeviceSVXStatus,
                'qsize': 10,
                'callback': self._statusCb
            }
        }


        # Create Node Class ####################

        self.con_node_if = ConnectNodeClassIF(
                        configs_dict = self.CONFIGS_DICT,
                        services_dict = self.SRVS_DICT,
                        pubs_dict = self.PUBS_DICT,
                        subs_dict = self.SUBS_DICT,
                        log_name_list = [],
                        msg_if = None
        )



        self.con_node_if.wait_for_ready()


        ##############################
        # Start updater process
        nepi_sdk.start_timer_process(1.0, self.updaterCb, oneshot = True)

        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete")
        ###############################


    #######################
    # Class Public Methods
    #######################


    def get_ready_state(self):
        """Return the ready state of the interface.

        Returns:
            bool: True if the interface has completed initialization, False otherwise.
        """
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        """Block until the interface is ready or the timeout expires.

        Args:
            timeout (float, optional): Maximum number of seconds to wait. Defaults to float('inf').

        Returns:
            bool: True if the interface became ready, False if the timeout was reached.
        """
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for connection")
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect")
            else:
                self.msg_if.pub_info("Connected")
        return self.ready

    def get_namespace(self):
        """Return the fully-resolved ROS namespace for the connected SVX device.

        Returns:
            str: The fully-qualified namespace string used for topic and service resolution.
        """
        return self.namespace

    def check_connection(self):
        """Check whether the device is currently connected.

        Returns:
            bool: True if a status message has been received within the connection timeout window,
                False otherwise.
        """
        return self.connected

    def wait_for_connection(self, timeout = float('inf') ):
        """Block until the device is connected or the timeout expires.

        Args:
            timeout (float, optional): Maximum number of seconds to wait. Defaults to float('inf').

        Returns:
            bool: True if connection was established, False if the timeout was reached.
        """
        if self.con_node_if is not None:
            self.msg_if.pub_info("Waiting for connection")
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.connected == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.connected == False:
                self.msg_if.pub_info("Failed to Connect")
            else:
                self.msg_if.pub_info("Connected")
        return self.connected


    def check_status_connection(self):
        """Check whether the status topic from the device is currently connected.

        Returns:
            bool: True if status messages are being received, False otherwise.
        """
        return self.connected

    def wait_for_status_connection(self, timeout = float('inf') ):
        """Block until the device status topic is connected or the timeout expires.

        Args:
            timeout (float, optional): Maximum number of seconds to wait. Defaults to float('inf').

        Returns:
            bool: True if the status connection was established, False if the timeout was reached.
        """
        if self.con_node_if is not None:
            self.msg_if.pub_info("Waiting for status connection")
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.connected == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.connected == False:
                self.msg_if.pub_info("Failed to connect to status msg")
            else:
                self.msg_if.pub_info("Status Connected")
        return self.connected

    def get_status_dict(self):
        """Return the latest device status as a dictionary.

        Returns:
            dict: A dictionary representation of the most recent DeviceSVXStatus message,
                or None if no status has been received yet.
        """
        status_dict = None
        if self.status_msg is not None:
            status_dict = nepi_sdk.convert_msg2dict(self.status_msg)
        return status_dict

    def get_status_msg(self):
        """Return the latest device status as a msg.

        Returns:
            DeviceSVXStatus: The most recent DeviceSVXStatus message, or None if no status
                has been received yet.
        """
        return self.status_msg

    def get_capabilities(self):
        """Query and return the SVX device capabilities report.

        Returns:
            SVXCapabilitiesQueryResponse: The capabilities response, or None if the call failed.
        """
        resp = None
        if self.con_node_if is not None:
            resp = self.con_node_if.call_service('capabilities_query', SVXCapabilitiesQueryRequest())
        return resp

    def get_servo_soft_limits(self):
        """Return the software softstop limits for the servo axis.

        Returns:
            list: A two-element list [min_deg, max_deg] representing the software softstop
                limits in degrees, or None if no status has been received.
        """
        if self.status_msg is not None:
            return [self.status_msg.min_softstop_deg, self.status_msg.max_softstop_deg]

    def get_servo_max_speed_dps(self):
        """Return the servo maximum speed in degrees per second.

        Returns:
            float: The maximum speed in degrees per second.
        """
        return self.speed_max_dps

    def get_servo_position(self):
        """Return the most recently reported servo position.

        Returns:
            float: The current position in degrees (last commanded value, open loop).
        """
        return self.position_deg

    def check_moving(self):
        """Check whether the servo is currently in motion.

        Returns:
            bool: True if the servo has moved more than 0.1 degrees since the last update
                cycle, False otherwise.
        """
        return self.moving

    def unregister(self):
        """Unregister all ROS subscribers and publishers for this device interface.

        Unsubscribes from all topics, marks the interface as disconnected, and releases
        the underlying node class resources.
        """
        self._unsubscribeTopic()

    def goto_position(self, position_deg):
        """Command the servo to move to an absolute position.

        Args:
            position_deg (float): Target position angle in degrees.
        """
        self.con_node_if.publish_pub('goto_position', float(position_deg))

    def goto_ratio(self, ratio):
        """Command the servo to move to a normalized ratio position.

        Args:
            ratio (float): Target position as a ratio from 0.0 to 1.0, where 0.0 corresponds
                to the minimum softstop and 1.0 to the maximum softstop.
        """
        self.con_node_if.publish_pub('goto_ratio', float(ratio))

    def set_soft_limits(self, min_deg, max_deg):
        """Set the software softstop limits for the servo axis.

        Args:
            min_deg (float): Minimum allowed position in degrees.
            max_deg (float): Maximum allowed position in degrees.
        """
        msg = ServoLimits()
        msg.min_deg = min_deg
        msg.max_deg = max_deg
        self.con_node_if.publish_pub('set_soft_limits', msg)

    def set_speed_ratio(self, speed_ratio):
        """Publish a speed ratio command to the servo.

        Args:
            speed_ratio (float): Desired motion speed as a ratio from 0.0 (slowest) to 1.0 (fastest).
        """
        self.con_node_if.publish_pub('set_speed_ratio', float(speed_ratio))

    def set_speed_max_dps(self, speed_max_dps):
        """Set the maximum servo speed in degrees per second.

        Args:
            speed_max_dps (float): Maximum speed in degrees per second (what a ratio of 1.0 means).
        """
        self.con_node_if.publish_pub('set_speed_max_dps', float(speed_max_dps))

    def set_reverse_enable(self, reverse_enable):
        """Enable or disable direction reversal on the servo.

        Args:
            reverse_enable (bool): True to reverse the axis direction, False for normal direction.
        """
        self.con_node_if.publish_pub('set_reverse_enable', bool(reverse_enable))

    def stop_moving(self):
        """Publish a stop command to halt motion on the servo.
        """
        self.con_node_if.publish_pub('stop_moving', Empty())

    def go_home(self):
        """Command the servo to move to its configured home position.
        """
        self.con_node_if.publish_pub('go_home', Empty())

    def set_home_position(self, position_deg):
        """Set the home position for the servo to a specified angle.

        Args:
            position_deg (float): Desired home position angle in degrees.
        """
        self.con_node_if.publish_pub('set_home_position', float(position_deg))

    def set_home_position_here(self):
        """Set the home position to the servo's current position.
        """
        self.con_node_if.publish_pub('set_home_position_here', Empty())

    def reset_device(self):
        """Command the servo device to reset to its default state.
        """
        self.con_node_if.publish_pub('reset_device', Empty())

    def set_spin_direction(self, spin_direction):
        """Set the spin direction for the servo.

        Args:
            spin_direction (int): Spin direction indicator (positive or negative).
        """
        self.con_node_if.publish_pub('set_spin_direction', int(spin_direction))

    def save_config(self):
        """Publish a save configuration command to persist current settings on the device.
        """
        self.con_node_if.publish_pub('save_config',Empty())

    def reset_config(self):
        """Publish a reset configuration command to restore the last saved settings on the device.
        """
        self.con_node_if.publish_pub('reset_config',Empty())

    def factory_reset_config(self):
        """Publish a factory reset command to restore factory default settings on the device.
        """
        self.con_node_if.publish_pub('factory_reset_config',Empty())


    ###############################
    # Class Private Methods
    ###############################

    def updaterCb(self,timer):
        cur_time = nepi_utils.get_time()
        last_time = copy.deepcopy(self.last_status_time )
        if self.connected == True:
            if (cur_time - last_time) > self.CONNECTED_TIMEOUT:
                self.connected = False
                self.status_msg = None
                self.moving = False

        if self.connected == True:
            self.moving = abs(self.position_deg - self.last_position_deg) > 0.1
            self.last_position_deg = self.position_deg

        nepi_sdk.start_timer_process(1.0, self.updaterCb, oneshot = True)

    def _unsubscribeTopic(self):
        success = False
        self.connected = False
        if self.con_node_if is not None:
            self.msg_if.pub_warn("Unregistering topic: " + str(self.namespace))
            try:
                self.con_node_if.unregister_class()
                time.sleep(1)
                self.con_node_if = None
                self.namespace = None
                self.connected = False
                self.status_msg = None
                success = True
            except Exception as e:
                self.msg_if.pub_warn("Failed to unregister svx:  " + str(e))
        return success


    def _statusCb(self,status_msg):
        self.last_status_time = nepi_utils.get_time()
        if self.connected == False:
            self.msg_if.pub_warn("Connected to SVX Status:  " + str(self.namespace))
        self.connected = True
        self.status_msg = status_msg
        self.speed_max_dps = status_msg.speed_max_dps
        self.position_deg = status_msg.position_now_deg
        if self.statusCb is not None:
            status_dict = self.get_status_dict()
            self.statusCb(status_dict)
