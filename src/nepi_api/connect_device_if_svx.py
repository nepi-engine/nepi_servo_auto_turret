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

import os
import time 
import copy 
import math
import numpy as np


from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64, Header
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from nepi_interfaces.msg import RangeWindow, NavPose, NavPoseServo, SaveDataRate
from nepi_interfaces.msg import DeviceSVXStatus, ServoLimits, ServoPosition, SingleAxisTimedMove, SingleAxisTimedSpeedMove
from nepi_interfaces.srv import SVXCapabilitiesQuery, SVXCapabilitiesQueryRequest, SVXCapabilitiesQueryResponse

from tf.transformations import quaternion_from_euler

from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF
from nepi_api.system_if import SaveDataIF, SettingsIF
from nepi_api.device_if_npx import NPXDeviceIF


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
    navpose_msg = None
    servo_position = [0,0]
    last_servo_position = [0,0]
    pan_moving = False
    tilt_moving = False

    statusCb = None
    navposeCb = None
    servoCb = None
    
    
    stopPanCb = None
    stopTiltCb = None

    speed_max_dps = 10
    #######################
    ### IF Initialization
    def __init__(self, 
                namespace = None,
                statusCb = None,
                servoCb = None,
                navposeCb = None,
                stopPanCb = None,
                stopTiltCb = None,
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
        self.msg_if.pub_info("Using PT Namespace: " + self.namespace)

        self.statusCb = statusCb
        self.navposeCb = navposeCb
        self.servoCb = servoCb


        self.stopPanCb = stopPanCb
        self.stopTiltCb = stopTiltCb


        ##############################   
        ## Node Setup

        # Configs Config Dict ####################
        self.CONFIGS_DICT = {
                'namespace': self.namespace
        }


        # Services Config Dict ####################
        self.SRVS_DICT = None


        # Publishers Config Dict ####################
        self.PUBS_DICT = {
            'speed_ratio': {
                'namespace': self.namespace,
                'topic': 'set_speed_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'pan_speed_ratio': {
                'namespace': self.namespace,
                'topic': 'set_pan_speed_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'tilt_speed_ratio': {
                'namespace': self.namespace,
                'topic': 'set_tilt_speed_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'stop_moving': {
                'namespace': self.namespace,
                'topic': 'stop_moving',
                'msg': Empty,
                'qsize': 1,
            },
            'goto_to_position': {
                'namespace': self.namespace,
                'topic': 'goto_position',
                'msg': ServoPosition,
                'qsize': 1,
            },
            'goto_to_pan_position': {
                'namespace': self.namespace,
                'topic': 'goto_pan_position',
                'msg': Float32,
                'qsize': 1,
            },
            'goto_to_tilt_position': {
                'namespace': self.namespace,
                'topic': 'goto_tilt_position',
                'msg': Float32,
                'qsize': 1,
            },
            'goto_pan_ratio': {
                'namespace': self.namespace,
                'topic': 'goto_pan_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'goto_tilt_ratio': {
                'namespace': self.namespace,
                'topic': 'goto_tilt_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'jog_timed_pan': {
                'namespace': self.namespace,
                'topic': 'jog_timed_pan',
                'msg': SingleAxisTimedMove,
                'qsize': 1,
            },
            'jog_timed_tilt': {
                'namespace': self.namespace,
                'topic': 'jog_timed_tilt',
                'msg': SingleAxisTimedMove,
                'qsize': 1,
            },
            'jog_timed_pan_speed_ratio': {
                'namespace': self.namespace,
                'topic': 'jog_timed_pan_speed_ratio',
                'msg': SingleAxisTimedSpeedMove,
                'qsize': 1,
            },
            'jog_timed_tilt_speed_ratio': {
                'namespace': self.namespace,
                'topic': 'jog_timed_tilt_speed_ratio',
                'msg': SingleAxisTimedSpeedMove,
                'qsize': 1,
            },
            'reverse_pan_enabled': {
                'namespace': self.namespace,
                'topic': 'set_reverse_pan_enable',
                'msg': Bool,
                'qsize': 1,
            },
            'reverse_tilt_enabled': {
                'namespace': self.namespace,
                'topic': 'set_reverse_tilt_enable',
                'msg': Bool,
                'qsize': 1,
            },
            'set_soft_limits': {
                'namespace': self.namespace,
                'topic': 'set_soft_limits',
                'msg': ServoLimits,
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
                'msg': ServoPosition,
                'qsize': 1,
            },
            'set_home_position_here': {
                'namespace': self.namespace,
                'topic': 'set_home_position_here',
                'msg': Empty,
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
            },
            'servo_position': {
                'namespace': self.namespace,
                'topic': 'servo',
                'msg': NavPoseServo,
                'qsize': 1,
                'callback': self._servoCb, 
                'callback_args': ()
            },
            'stop_pan_callback': {
                'topic': 'stop_pan_callback',
                'msg': Empty,
                'namespace': self.namespace,
                'qsize': 5,
                'callback': self._stopPanCb, 
                'callback_args': ()
            },
            'stop_tilt_callback': {
                'msg': Empty,
                'namespace': self.namespace,
                'topic': 'stop_tilt_callback',
                'qsize': 5,
                'callback': self._stopTiltCb, 
                'callback_args': ()
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

    def wait_for_ready(self, timout = float('inf') ):
        """Block until the interface is ready or the timeout expires.

        Args:
            timout (float, optional): Maximum number of seconds to wait. Defaults to float('inf').

        Returns:
            bool: True if the interface became ready, False if the timeout was reached.
        """
        success = False
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

    def wait_for_connection(self, timout = float('inf') ):
        """Block until the device is connected or the timeout expires.

        Args:
            timout (float, optional): Maximum number of seconds to wait. Defaults to float('inf').

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

    def wait_for_status_connection(self, timout = float('inf') ):
        """Block until the device status topic is connected or the timeout expires.

        Args:
            timout (float, optional): Maximum number of seconds to wait. Defaults to float('inf').

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
            dict: A msg representation of the most recent DeviceSVXStatus message,
                or None if no status has been received yet.
        """
        return self.status_msg

    # def get_navpose_dict(self):
    #     navpose_dict = None
    #     if self.navpose_msg is not None:
    #         navpose_dict = nepi_nav.convert_navpose_msg2dict(self.navpose_msg)
    #     return navpose_dict
    
    def get_servo_hard_limits(self):
        """Return the hardware hardstop limits for servo axes.

        Returns:
            list: A four-element list [pan_min_deg, pan_max_deg, tilt_min_deg, tilt_max_deg]
                representing the physical hardstop limits in degrees, or None if no status
                has been received.
        """
        if self.status_msg is not None:
            return [self.status_msg.pan_min_hardstop_deg, self.status_msg.pan_max_hardstop_deg, self.status_msg.tilt_min_hardstop_deg, self.status_msg.pan_max_hardstop_deg]
    
    def get_servo_soft_limits(self):
        """Return the software softstop limits for servo axes.

        Returns:
            list: A four-element list [pan_min_deg, pan_max_deg, tilt_min_deg, tilt_max_deg]
                representing the software softstop limits in degrees, or None if no status
                has been received.
        """
        if self.status_msg is not None:
            return [self.status_msg.pan_min_softstop_deg, self.status_msg.pan_max_softstop_deg, self.status_msg.tilt_min_softstop_deg, self.status_msg.tilt_max_softstop_deg]

    def get_servo_max_speed_dps(self):
        return self.speed_max_dps
    

    def get_servo_position(self):
        """Return the most recently reported servo position.

        Returns:
            list: A two-element list [pan_deg, tilt_deg] with the current position in degrees.
        """
        return [self.servo_position[0], self.servo_position[1]]

    def check_pan_moving(self):
        """Check whether the pan axis is currently in motion.

        Returns:
            bool: True if the pan axis has moved more than 0.1 degrees since the last update
                cycle, False otherwise.
        """
        return self.pan_moving
    
    def check_tilt_moving(self):
        """Check whether the tilt axis is currently in motion.

        Returns:
            bool: True if the tilt axis has moved more than 0.1 degrees since the last update
                cycle, False otherwise.
        """
        return self.tilt_moving

    def unregister(self):
        """Unregister all ROS subscribers and publishers for this device interface.

        Unsubscribes from all topics, marks the interface as disconnected, and releases
        the underlying node class resources.
        """
        self._unsubscribeTopic()

    def set_speed_ratio(self, speed_ratio):
        """Publish a speed ratio command to the SVX device.

        Args:
            speed_ratio (float): Desired motion speed as a ratio from 0.0 (slowest) to 1.0 (fastest).
        """
        pub_name = 'speed_ratio'
        msg = speed_ratio
        self.con_node_if.publish_pub(pub_name, msg)

    def set_pan_speed_ratio(self, speed_ratio):
        """Publish a pan-axis speed ratio command to a device that supports separate servo speed control.

        Args:
            speed_ratio (float): Desired pan speed as a ratio from 0.0 (slowest) to 1.0 (fastest).
        """
        pub_name = 'pan_speed_ratio'
        msg = speed_ratio
        self.con_node_if.publish_pub(pub_name, msg)

    def set_tilt_speed_ratio(self, speed_ratio):
        """Publish a tilt-axis speed ratio command to a device that supports separate servo speed control.

        Args:
            speed_ratio (float): Desired tilt speed as a ratio from 0.0 (slowest) to 1.0 (fastest).
        """
        pub_name = 'tilt_speed_ratio'
        msg = speed_ratio
        self.con_node_if.publish_pub(pub_name, msg)

    def stop_moving(self):
        """Publish a stop command to halt all motion on the SVX device.
        """
        pub_name = 'stop_moving'
        msg = Empty()
        self.con_node_if.publish_pub(pub_name,msg)


    def goto_to_position(self,pan_deg,tilt_deg):
        """Command the SVX device to move to an absolute servo position.

        Args:
            pan_deg (float): Target pan angle in degrees.
            tilt_deg (float): Target tilt angle in degrees.
        """
        pub_name = 'goto_to_position'
        msg = ServoPosition()
        msg.pan_deg = pan_deg
        msg.tilt_deg = tilt_deg
        self.con_node_if.publish_pub(pub_name,msg)

    def goto_to_pan_position(self,pan_position):
        """Command the SVX device to move the pan axis to an absolute position.

        Args:
            pan_position (float): Target pan angle in degrees.
        """
        #self.msg_if.pub_warn("connect goto pan pos: " + str(pan_position))

        pub_name = 'goto_to_pan_position'
        msg = pan_position
        self.con_node_if.publish_pub(pub_name,msg)
        
    def goto_to_tilt_position(self,tilt_position):
        """Command the SVX device to move the tilt axis to an absolute position.

        Args:
            tilt_position (float): Target tilt angle in degrees.
        """
        pub_name = 'goto_to_tilt_position'
        msg = tilt_position
        self.con_node_if.publish_pub(pub_name,msg)

    def goto_pan_ratio(self,pan_ratio):
        """Command the SVX device to move the pan axis to a normalized ratio position.

        Args:
            pan_ratio (float): Target pan position as a ratio from 0.0 to 1.0, where 0.0
                corresponds to the minimum softstop and 1.0 to the maximum softstop.
        """
        pub_name = 'goto_pan_ratio'
        msg = pan_ratio
        self.con_node_if.publish_pub(pub_name,msg)        

    def goto_tilt_ratio(self, tilt_ratio):
        """Command the SVX device to move the tilt axis to a normalized ratio position.

        Args:
            tilt_ratio (float): Target tilt position as a ratio from 0.0 to 1.0, where 0.0
                corresponds to the minimum softstop and 1.0 to the maximum softstop.
        """
        pub_name = 'goto_tilt_ratio'
        msg = tilt_ratio
        self.con_node_if.publish_pub(pub_name,msg)

    def jog_timed_pan(self, direction, duration_s = -1):
        """Command the SVX device to jog the pan axis in a direction for a set duration.

        Args:
            direction (int): Direction indicator for the pan jog (positive or negative).
            duration_s (float): Duration of the jog in seconds. Pass -1.0 for an indefinite move.
        """
        pub_name = 'jog_timed_pan'
        msg = SingleAxisTimedMove()
        # Direction indicator
        msg.direction = direction
        # Duration, -1.0 for infinite duration
        msg.duration_s = duration_s
        self.con_node_if.publish_pub(pub_name,msg)
        
    def jog_timed_tilt(self, direction, duration_s = -1):
        """Command the SVX device to jog the tilt axis in a direction for a set duration.

        Args:
            direction (int): Direction indicator for the tilt jog (positive or negative).
            duration_s (float): Duration of the jog in seconds. Pass -1.0 for an indefinite move.
        """
        pub_name = 'jog_timed_tilt'
        msg = SingleAxisTimedMove()
        # Direction indicator
        msg.direction = direction
        # Duration, -1.0 for infinite duration
        msg.duration_s = duration_s
        self.con_node_if.publish_pub(pub_name,msg) 

    def jog_timed_speed_ratio_pan(self, direction, speed_ratio = 1, duration_s = -1):
        """Command the SVX device to jog the pan axis in a direction at speed_ratio for a set duration.

        Args:
            direction (int): Direction indicator for the pan jog (positive or negative).
            duration_s (float): Duration of the jog in seconds. Pass -1.0 for an indefinite move.
        """
        pub_name = 'jog_timed_pan_speed_ratio'
        msg = SingleAxisTimedSpeedMove()
        # Direction indicator
        msg.direction = direction
        msg.speed_ratio = speed_ratio
        # Duration, -1.0 for infinite duration
        msg.duration_s = duration_s
        self.con_node_if.publish_pub(pub_name,msg)
        
    def jog_timed_speed_ratio_tilt(self, direction, speed_ratio = 1, duration_s = -1):
        """Command the SVX device to jog the tilt axis in a direction speed_ratio for a set duration.

        Args:
            direction (int): Direction indicator for the tilt jog (positive or negative).
            duration_s (float): Duration of the jog in seconds. Pass -1.0 for an indefinite move.
        """
        pub_name = 'jog_timed_tilt_speed_ratio'
        msg = SingleAxisTimedSpeedMove()
        # Direction indicator
        msg.direction = direction
        msg.speed_ratio = speed_ratio
        # Duration, -1.0 for infinite duration
        msg.duration_s = duration_s
        self.con_node_if.publish_pub(pub_name,msg) 


    def jog_timed_speed_dps_pan(self, direction, speed_dps = 1, duration_s = -1):
        """Command the SVX device to jog the pan axis in a direction at speed_dps for a set duration.

        Args:
            direction (int): Direction indicator for the pan jog (positive or negative).
            duration_s (float): Duration of the jog in seconds. Pass -1.0 for an indefinite move.
        """
        pub_name = 'jog_timed_pan_speed_ratio'
        msg = SingleAxisTimedSpeedMove()
        # Direction indicator
        msg.direction = direction

        if speed_dps < 0:
            speed_dps = 0
        if speed_dps > self.speed_max_dps:
            speed_dps = self.speed_max_dps
        msg.speed_ratio = speed_dps / self.speed_max_dps
        # Duration, -1.0 for infinite duration
        msg.duration_s = duration_s
        self.con_node_if.publish_pub(pub_name,msg)
        
    def jog_timed_speed_dps_tilt(self, direction, speed_dps = 1, duration_s = -1):
        """Command the SVX device to jog the tilt axis in a direction speed_dps for a set duration.

        Args:
            direction (int): Direction indicator for the tilt jog (positive or negative).
            duration_s (float): Duration of the jog in seconds. Pass -1.0 for an indefinite move.
        """
        pub_name = 'jog_timed_tilt_speed_ratio'
        msg = SingleAxisTimedSpeedMove()
        # Direction indicator
        msg.direction = direction
        if speed_dps < 0:
            speed_dps = 0
        if speed_dps > self.speed_max_dps:
            speed_dps = self.speed_max_dps
        msg.speed_ratio = speed_dps / self.speed_max_dps
        # Duration, -1.0 for infinite duration
        msg.duration_s = duration_s
        self.con_node_if.publish_pub(pub_name,msg) 


    def reverse_pan_enabled(self, reverse_pan):
        """Enable or disable pan direction reversal on the SVX device.

        Args:
            reverse_pan (bool): True to reverse the pan axis direction, False for normal direction.
        """
        pub_name = 'reverse_pan_enabled'
        msg = reverse_pan
        self.con_node_if.publish_pub(pub_name,msg)

    def reverse_tilt_enabled(self, reverse_tilt):
        """Enable or disable tilt direction reversal on the SVX device.

        Args:
            reverse_tilt (bool): True to reverse the tilt axis direction, False for normal direction.
        """
        pub_name = 'reverse_tilt_enabled'
        msg = reverse_tilt
        self.con_node_if.publish_pub(pub_name,msg)

    def go_home(self):
        """Command the SVX device to move to its configured home position.
        """
        pub_name = 'go_home'
        msg = Empty()
        self.con_node_if.publish_pub(pub_name,msg)

    def set_home_position(self,pan_deg,tilt_deg):
        """Set the home position for the SVX device to specified servo angles.

        Args:
            pan_deg (float): Desired home pan angle in degrees.
            tilt_deg (float): Desired home tilt angle in degrees.
        """
        pub_name = 'set_home_position'
        msg = ServoPosition()
        msg.pan_deg = pan_deg
        msg.tilt_deg = tilt_deg
        self.con_node_if.publish_pub(pub_name,msg) 

    def set_home_position_here(self):
        """Set the home position to the SVX device's current position.
        """
        pub_name = 'set_home_position_here'
        msg = Empty()
        self.con_node_if.publish_pub(pub_name,msg)  

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

    #################
    ## Save Data Functions

    def get_save_data_products(self):
        """Return the list of available save data products for this device.

        Returns:
            list: A list of data product identifiers supported by the save data interface.
        """
        data_products = self.con_save_data_if.get_data_products()
        return data_products

    def get_save_data_status_dict(self):
        """Return the current save data status as a dictionary.

        Returns:
            dict: A dictionary representation of the save data interface status.
        """
        status_dict = self.con_save_data_if.get_status_dict()
        return status_dict

    def save_data_enable_pub(self,enable):
        """Enable or disable data saving on the device.

        Args:
            enable (bool): True to enable data saving, False to disable it.
        """
        self.con_save_data_if.save_data_pub(enable)

    def save_data_prefix_pub(self,prefix):
        """Publish an updated filename prefix for saved data files.

        Args:
            prefix (str): The prefix string to prepend to saved data filenames.
        """
        self.con_save_data_if.save_data_prefix_pub(prefix)

    def save_data_rate_pub(self,rate_hz, data_product = SaveDataRate.ALL_DATA_PRODUCTS):
        """Publish an updated save rate for a data product.

        Args:
            rate_hz (float): Desired save rate in Hz.
            data_product (int, optional): Identifier for the specific data product to update.
                Defaults to SaveDataRate.ALL_DATA_PRODUCTS.
        """
        self.con_save_data_if.publish_pub(rate_hz, data_product = SaveDataRate.ALL_DATA_PRODUCTS)

    def save_data_snapshot_pub(self):
        """Trigger a one-shot snapshot save of current data on the device.
        """
        self.con_save_data_if.publish_pub()

    def save_data_reset_pub(self):
        """Publish a reset command to clear saved data state on the device.
        """
        self.con_save_data_if.publish_pub(pub_name,msg)

    def save_data_factory_reset_pub(self):
        """Publish a factory reset command to restore the save data configuration to defaults.
        """
        pub_name = 'factory_reset'
        msg = Empty()
        self.con_save_data_if.publish_pub(pub_name,msg)

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
                self.navpose_msg = None
                self.data_dict = None
                self.pan_moving = False
                self.tilt_moving = False

        if self.connected == True:
            self.pan_moving = abs(self.servo_position[0] - self.last_servo_position[0]) > 0.1
            self.tilt_moving = abs(self.servo_position[1] - self.last_servo_position[1]) > 0.1
  
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
                self.navpose_msg = None
                self.data_dict = None
                success = True
            except Exception as e:
                self.msg_if.pub_warn("Failed to unregister image:  " + str(e))
        return success


    def _statusCb(self,status_msg):  
        self.last_status_time = nepi_utils.get_time()
        if self.connected == False:
            self.msg_if.pub_warn("Connected to PT Status:  " + str(self.namespace))
        self.connected = True
        self.status_msg = status_msg
        self.speed_max_dps = status_msg.speed_max_dps
        if self.statusCb is not None:
            status_dict = self.get_status_dict()
            self.statusCb(status_dict)

    def _servoCb(self,servo_msg):    
        #self.msg_if.pub_warn("Connected to PT Position")  
        self.servo_position = [servo_msg.pan_deg, servo_msg.tilt_deg]
        if self.servoCb is not None:
            #self.msg_if.pub_warn("servoCb: " + str(self.servoCb))
            self.servoCb(self.servo_position[0], self.servo_position[1])
    

    def _stopPanCb(self,msg):    
        self.msg_if.pub_warn("Got Stop Pan msg: " + str(msg))
        if self.stopPanCb is not None:
            self.stopPanCb()

    def _stopTiltCb(self,msg):    
        self.msg_if.pub_warn("Got Stop Tilt msg: " + str(msg))
        if self.stopTiltCb is not None:
            self.stopTiltCb()