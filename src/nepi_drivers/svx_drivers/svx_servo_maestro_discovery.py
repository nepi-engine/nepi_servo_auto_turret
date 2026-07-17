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

# SVX (servo) discovery for the Pololu Micro Maestro USB servo controller.
#
# Structure mirrors svx_servo_generic_discovery.py (the SVX template) and the
# serial discovery pattern in ptx_sidus_ss109_serial_discovery.py. The stub
# detection in the generic discovery is replaced here with a real USB match on
# the Pololu vendor/product IDs.
#
# One SVX device = one servo = one Maestro channel. A Maestro is a single USB
# device exposing two virtual serial ports (a Command Port and a TTL Port); we
# send native commands to the Command Port. For each detected board we launch one
# SVX node per configured servo channel, all sharing that one Command Port. The
# node serializes port access with an advisory file lock, so several channel
# nodes on one board coexist safely.

import time

import serial
from serial.tools import list_ports

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_drvs
from nepi_sdk import nepi_system

PKG_NAME = 'SVX_SERVO_MAESTRO'
FILE_TYPE = 'DISCOVERY'


class SvxServoMaestroDiscovery:

    # Pololu USB vendor ID and the Maestro product IDs (Micro 6 + Mini 12/18/24).
    # Both virtual serial ports of a board share the same VID/PID; they differ by
    # USB interface number, which is how we pick the Command Port below.
    POLOLU_VENDOR_ID = 0x1FFB
    MAESTRO_PRODUCT_IDS = [0x0089, 0x008A, 0x008B, 0x008C]

    node_launch_name = "maestro"

    # launch_id ("<port>:ch<N>") -> {node_name, sub_process, path, channel}
    active_devices_dict = dict()
    active_paths_list = []
    dont_retry_list = []

    retry = True

    # Discovery options (populated from drv_dict each pass)
    channels_list = [0]
    baud_str = '9600'
    device_number = 12
    protocol = 'Compact'
    command_port_index = 0
    pulse_min_us = 1000.0
    pulse_max_us = 2000.0
    min_deg = -90.0
    max_deg = 90.0
    accel_units = 0


    ################################################
    def __init__(self):
        self.log_name = PKG_NAME.lower() + "_discovery"
        self.logger = nepi_sdk.logger(log_name = self.log_name)
        time.sleep(1)
        self.logger.log_info("Starting Initialization")
        self.logger.log_info("Initialization Complete")


    ##########  DRV Standard Discovery Function
    def discoveryFunction(self, available_paths_list, active_paths_list, base_namespace, drv_dict, retry_enabled = True):
        self.drv_dict = drv_dict
        self.available_paths_list = available_paths_list
        self.active_paths_list = active_paths_list
        self.base_namespace = base_namespace

        ##################################
        # Get discovery options
        try:
            options = self.drv_dict.get('DISCOVERY_DICT', {}).get('OPTIONS', {})
            self.channels_list = self.parseChannels(options.get('channels', {}).get('value', '0'))
            self.baud_str = str(options.get('baud_rate', {}).get('value', '9600'))
            self.device_number = int(options.get('device_number', {}).get('value', 12))
            self.protocol = str(options.get('protocol', {}).get('value', 'Compact'))
            self.command_port_index = int(options.get('command_port_index', {}).get('value', 0))
            self.pulse_min_us = float(options.get('pulse_min_us', {}).get('value', 1000.0))
            self.pulse_max_us = float(options.get('pulse_max_us', {}).get('value', 2000.0))
            self.min_deg = float(options.get('min_deg', {}).get('value', -90.0))
            self.max_deg = float(options.get('max_deg', {}).get('value', 90.0))
            self.accel_units = int(options.get('accel_units', {}).get('value', 0))
        except Exception as e:
            self.logger.log_warn(self.log_name + ": Failed to setup options " + str(e))
            return self.active_paths_list

        # Retry behavior
        self.retry = retry_enabled
        if self.retry == True:
            self.dont_retry_list = []

        ### Purge nodes whose Command Port has disappeared
        path_purge_list = []
        for launch_id in list(self.active_devices_dict.keys()):
            path_str = self.active_devices_dict[launch_id]['path']
            if path_str not in self.available_paths_list:
                path_purge_list.append(launch_id)
        for launch_id in path_purge_list:
            self.killDevice(launch_id)

        ### Look for Maestro Command Ports and launch a node per configured channel
        command_ports = self.findCommandPorts()
        for port_info in command_ports:
            path_str = port_info['path']
            if path_str in self.active_paths_list or path_str in self.dont_retry_list:
                continue
            self.logger.log_info("Found Maestro Command Port at: " + path_str)
            launched_any = False
            for channel in self.channels_list:
                success = self.launchDeviceNode(path_str, channel, port_info['serial_number'])
                launched_any = launched_any or success
            if launched_any and path_str not in self.active_paths_list:
                self.active_paths_list.append(path_str)
        return self.active_paths_list
    ################################################


    ##########  Device specific calls

    def parseChannels(self, channels_str):
        # "0" -> [0]; "0,1" -> [0,1]; "all" -> [0..5] (Micro Maestro 6-channel).
        channels_str = str(channels_str).strip().lower()
        if channels_str == 'all':
            return list(range(0, 6))
        channels = []
        for tok in channels_str.replace(';', ',').split(','):
            tok = tok.strip()
            if tok == '':
                continue
            try:
                ch = int(tok)
                if 0 <= ch <= 23 and ch not in channels:
                    channels.append(ch)
            except Exception:
                pass
        if len(channels) == 0:
            channels = [0]
        return channels


    def findCommandPorts(self):
        # Enumerate USB serial ports, keep the Pololu Maestro matches, group the
        # two ports of each physical board by serial number, and return the
        # Command Port of each board (the lowest USB interface number, or the
        # command_port_index-th when a board exposes them in a fixed order).
        matches = []
        try:
            for p in list_ports.comports():
                vid = getattr(p, 'vid', None)
                pid = getattr(p, 'pid', None)
                if vid == self.POLOLU_VENDOR_ID and (pid in self.MAESTRO_PRODUCT_IDS or pid is None):
                    matches.append(p)
        except Exception as e:
            self.logger.log_warn("Failed to enumerate serial ports: " + str(e))
            return []

        # Group by serial number (falls back to the location prefix if the board
        # reports no serial number).
        boards = dict()
        for p in matches:
            key = getattr(p, 'serial_number', None)
            if key is None:
                loc = getattr(p, 'location', None) or getattr(p, 'device', '')
                key = str(loc).split(':')[0]
            boards.setdefault(key, []).append(p)

        command_ports = []
        for key, ports in boards.items():
            ports_sorted = sorted(ports, key = self.interfaceSortKey)
            idx = self.command_port_index
            if idx < 0 or idx >= len(ports_sorted):
                idx = 0
            chosen = ports_sorted[idx]
            command_ports.append({
                'path': chosen.device,
                'serial_number': getattr(chosen, 'serial_number', None) or 'Unknown'
            })
        return command_ports


    def interfaceSortKey(self, port):
        # Sort a board's ports by USB interface number so the Command Port
        # (interface 0) sorts first. location looks like "1-1.2:1.0"; the trailing
        # ".0" is the interface. Falls back to the device string.
        loc = getattr(port, 'location', None)
        if loc is not None and ':' in loc:
            try:
                return (0, int(loc.split(':')[-1].split('.')[-1]))
            except Exception:
                pass
        return (1, str(getattr(port, 'device', '')))


    def launchDeviceNode(self, path_str, channel, serial_number):
        launch_id = path_str + ":ch" + str(channel)
        if launch_id in self.active_devices_dict:
            return True

        file_name = self.drv_dict['NODE_DICT']['file_name']
        device_name = self.node_launch_name + "_" + path_str.split('/')[-1] + "_ch" + str(channel)
        node_name = nepi_system.get_device_alias(device_name)
        self.logger.log_info("Launching node: " + node_name + " on channel " + str(channel))

        # Setup required param server drv_dict for the node
        dict_param_name = nepi_sdk.create_namespace(self.base_namespace, node_name + "/drv_dict")
        self.drv_dict['DEVICE_DICT'] = {
            'device_name': device_name,
            'device_path': path_str,
            'channel': channel,
            'baud_str': self.baud_str,
            'device_number': self.device_number,
            'serial_number': serial_number
        }
        nepi_sdk.set_param(dict_param_name, self.drv_dict)

        [success, msg, sub_process] = nepi_drvs.launchDriverNode(file_name, node_name, device_path = path_str)
        if success:
            self.active_devices_dict[launch_id] = {
                'node_name': node_name,
                'sub_process': sub_process,
                'path': path_str,
                'channel': channel
            }
            self.logger.log_info("Launched node: " + node_name)
        else:
            self.logger.log_info("Failed to launch node: " + node_name + " with msg: " + msg)
            if self.retry == False:
                self.dont_retry_list.append(path_str)
        return success


    def killDevice(self, launch_id):
        if launch_id not in self.active_devices_dict:
            return
        entry = self.active_devices_dict[launch_id]
        node_name = entry['node_name']
        sub_process = entry['sub_process']
        path_str = entry['path']
        self.logger.log_info("No longer detecting Maestro; killing node: " + node_name)
        nepi_drvs.killDriverNode(node_name, sub_process)
        del self.active_devices_dict[launch_id]
        # Drop the path from active list only once none of its channels remain.
        still_active = any(e['path'] == path_str for e in self.active_devices_dict.values())
        if not still_active and path_str in self.active_paths_list:
            self.active_paths_list.remove(path_str)


    def killAllDevices(self, active_paths_list):
        for launch_id in list(self.active_devices_dict.keys()):
            entry = self.active_devices_dict[launch_id]
            node_name = entry['node_name']
            sub_process = entry['sub_process']
            path_str = entry['path']
            if self.retry == False:
                self.dont_retry_list.append(path_str)
            nepi_drvs.killDriverNode(node_name, sub_process)
            if path_str in active_paths_list:
                active_paths_list.remove(path_str)
            if path_str in self.active_paths_list:
                self.active_paths_list.remove(path_str)
        self.active_devices_dict = dict()
        nepi_sdk.sleep(1)
        return active_paths_list


if __name__ == '__main__':
    SvxServoMaestroDiscovery()
