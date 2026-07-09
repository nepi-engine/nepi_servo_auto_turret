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


#import os
import copy
# import numpy as np
# import cv2
# import math
# import pandas as pd
# from scipy.stats import linregress
# from scipy.signal import medfilt
# from scipy.interpolate import UnivariateSpline
# from scipy.interpolate import CubicSpline


from nepi_interfaces.msg import NavPose, NavPoseOrientation

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_nav



from nepi_sdk.nepi_sdk import logger as Logger
log_name = "nepi_stab"
logger = Logger(log_name = log_name)


SOURCE_MESSAGE_DICT = {'NavPose' : NavPose, 'NavPoseOrientation': NavPoseOrientation}

DATA_DICT = {
    # Required Fields
    'data_time': 0.0,
    'process_time': 0.0,

    'roll_deg': -999,
    'rolls_dps': [],
    'roll_dps': -999,

    'pitch_deg': -999,
    'pitchs_dps': [],
    'pitch_dps': -999,

    'heading_deg': -999,
    'headings_dps': [],
    'heading_dps': -999,

    'servo_max_speed_dps': 10,
    'pan_deg': 0.0,
    'pan_dps': 0.0,
    'pan_speed_start_ratio': 1.0,
    'pan_min_deg': -170,
    'pan_max_deg': 170,


    'tilt_deg': 0.0,
    'tilt_dps': 0.0,
    'tilt_speed_start_ratio': 1.0,
    'tilt_min_deg': -50,
    'tilt_max_deg': 50,


    'stab_pan_deg': 0.0,
    'stab_pan_adjs': [],
    'stab_pan_adj': 0.0,
    'stab_pan_goal': 0.0,
    'stab_pan_dir': 0,
    'stab_pan_dps': 0.0,
    'stab_pan_vel_rate': 0.0,
    'stab_pan_pos_rate': 0.0,  

    'stab_tilt_deg': 0.0,
    'stab_tilt_adjs': [],
    'stab_tilt_adj': 0.0,
    'stab_tilt_goal': 0.0,
    'stab_tilt_dir': 0,
    'stab_tilt_dps': 0.0,
    'stab_tilt_vel_rate': 0.0,
    'stab_tilt_pos_rate': 0.0,    

    'last_stab_time': None,
    'last_pan_vel_time': 0,
    'last_pan_pos_time': 0,
    'last_tilt_vel_time': 0,
    'last_tilt_pos_time': 0,
     # Add Custom Fields Here

}


PROCESSES_DICT = dict()

DEFAULT_PROCESS = 'sv_stab_1'

#########################
# Stab Process Functions
#########################



sv_stab_1_settings = {
    # Required Fields
    'stab_update_rate': 5,
    'stab_reset_time_sec': 3.0,
    'stab_num_avg': 2,
    'stab_sv_min_speed_ratio': 0.1,
    'stab_sv_max_speed_ratio': 0.9,

    # Custom Fields. Automatically Populated in RUI
    'stab_controls_dict': {
        'pos_deg': 2.0,
        'vel_deg': 20.0,
    }
}

def sv_stab_1(sv_connect_if, 
                        stab_data_dict, 
                        stab_settings_dict, 
                        goto_position,
                        stab_pan_enabled, 
                        stab_tilt_enabled
                        ):

    #logger.log_info("******")
    #logger.log_info("*** Stabs Solution Update Starting ***")
    #logger.log_info("******")
    start_time = nepi_utils.get_time()
    ##########################
    # Gather Stab Settings
    ##########################
    [pan_goal, tilt_goal] = goto_position

    pan_deg = stab_data_dict['pan_deg']
    pan_dps = stab_data_dict['pan_dps']
    pan_speed_start_ratio = stab_data_dict['pan_speed_start_ratio']
    tilt_deg = stab_data_dict['tilt_deg']
    tilt_dps = stab_data_dict['tilt_dps']
    tilt_speed_start_ratio = stab_data_dict['tilt_speed_start_ratio']
    servo_max_speed_dps = stab_data_dict['servo_max_speed_dps']

    stab_min_dps = stab_settings_dict['stab_sv_min_speed_ratio'] * servo_max_speed_dps 
    stab_max_dps = stab_settings_dict['stab_sv_max_speed_ratio'] * servo_max_speed_dps
    controls_dict = stab_settings_dict['stab_controls_dict']

  
    ######################
    ## Process Stab Tilt Adjustments
    ######################
    pos_deg = controls_dict['pos_deg']
    vel_deg = controls_dict['vel_deg']

    last_data_dict = copy.deepcopy(stab_data_dict)
    last_tilt_dps = last_data_dict['stab_tilt_dps']
    last_tilt_goal = last_data_dict['stab_tilt_goal']
    last_tilt_goal_delta = (last_tilt_goal - tilt_deg)
    last_tilt_dir = last_data_dict['stab_tilt_dir']


    tilt_adj = stab_data_dict['stab_tilt_adj']
    adj_tilt_goal = round(tilt_goal + tilt_adj,1)
    adj_tilt_goal_delta = (adj_tilt_goal - tilt_deg)
    adj_tilt_dir = 1 if adj_tilt_goal_delta > 0 else -1
    adj_tilt_change = (last_tilt_goal - adj_tilt_goal)

    
    if stab_tilt_enabled == True:
        # Process Velocity Adjustment
        if abs(adj_tilt_goal_delta) > vel_deg : # or abs(adj_tilt_goal_delta) < pos_deg:
            stab_speed_ratio = tilt_speed_start_ratio
            stab_tilt_dps = round(stab_speed_ratio * servo_max_speed_dps,1)
        else:
            stab_tilt_dps = round(stab_min_dps + (stab_max_dps - stab_min_dps) * abs(adj_tilt_goal_delta) / vel_deg, 1)
            stab_speed_ratio = stab_tilt_dps / servo_max_speed_dps

        if abs(stab_tilt_dps - last_tilt_dps) > 1:   
            last_time = stab_data_dict['last_tilt_vel_time']
            update_rate = float(1) / (start_time - last_time)
            stab_data_dict['last_tilt_vel_time'] = start_time
            stab_data_dict['stab_tilt_vel_rate'] = round(update_rate,2) 
            stab_data_dict['stab_tilt_dps'] = stab_tilt_dps     
            sv_connect_if.set_tilt_speed_ratio(stab_speed_ratio)

        # Process Position Adjustment
        pos_needs_update = False
        if (last_tilt_dir != adj_tilt_dir):
            pos_needs_update = True
        else:
            #logger.log_warn("Stab Tilt Pos Check: " + str([abs(adj_tilt_goal_delta), pos_deg, abs(adj_tilt_change), pos_deg]) ) 
            #logger.log_warn("Stab Tilt Pos Check: " + str([abs(adj_tilt_goal_delta) > pos_deg, abs(adj_tilt_change) > pos_deg]) ) 
            pos_needs_update = abs(adj_tilt_goal_delta) > pos_deg or abs(adj_tilt_change) > pos_deg
            if adj_tilt_dir == 1:
                pos_needs_update = pos_needs_update and \
                                    adj_tilt_goal > last_tilt_goal and \
                                    abs(adj_tilt_goal - last_tilt_goal) > pos_deg 
            else:
                pos_needs_update = pos_needs_update and \
                                    adj_tilt_goal < last_tilt_goal and \
                                    abs(adj_tilt_goal - last_tilt_goal) > pos_deg 
        if pos_needs_update == True:
            last_time = stab_data_dict['last_tilt_pos_time']
            update_rate = float(1) / (start_time - last_time)
            stab_data_dict['last_tilt_pos_time'] = start_time
            stab_data_dict['stab_tilt_pos_rate'] = round(update_rate,2)
            stab_data_dict['stab_tilt_goal'] = adj_tilt_goal
            stab_data_dict['stab_tilt_dir'] = adj_tilt_dir
            sv_connect_if.goto_to_tilt_position(adj_tilt_goal)
            #logger.log_info("Stab Tilt Position updated: " + str([adj_tilt_goal_delta, stab_speed_ratio, adj_tilt_goal]))

            
    return stab_data_dict, stab_settings_dict



PROCESSES_DICT['sv_stab_1'] = {'process_function': sv_stab_1, 
                                             'default_settings_dict': sv_stab_1_settings}


############################

sv_stab_2_settings = {
    # Required Fields
    'stab_update_rate': 5,
    'stab_reset_time_sec': 3.0,
    'stab_num_avg': 2,
    'stab_sv_min_speed_ratio': 0.1,
    'stab_sv_max_speed_ratio': 0.9,

    # Custom Fields. Automatically Populated in RUI
    'stab_controls_dict': {
        'pos_deg': 2.0,
        'vel_deg': 20.0,
    }
}

def sv_stab_2(sv_connect_if,
                        stab_data_dict,
                        stab_settings_dict,
                        goto_position,
                        stab_pan_enabled,
                        stab_tilt_enabled
                        ):
    # Proportional velocity controller — drives axes via jog_timed_speed_dps instead of
    # position commands. Velocity is proportional to angular error; motion stops when
    # error falls within the pos_deg dead zone.
    start_time = nepi_utils.get_time()
    [pan_goal, tilt_goal] = goto_position

    tilt_deg = stab_data_dict['tilt_deg']
    servo_max_speed_dps = stab_data_dict['servo_max_speed_dps']
    last_tilt_dps = stab_data_dict['stab_tilt_dps']
    last_tilt_dir = stab_data_dict['stab_tilt_dir']

    stab_min_dps = stab_settings_dict['stab_sv_min_speed_ratio'] * servo_max_speed_dps
    stab_max_dps = stab_settings_dict['stab_sv_max_speed_ratio'] * servo_max_speed_dps
    controls_dict = stab_settings_dict['stab_controls_dict']
    pos_deg = controls_dict['pos_deg']
    vel_deg = controls_dict['vel_deg']

    tilt_adj = stab_data_dict['stab_tilt_adj']
    adj_tilt_goal = round(tilt_goal + tilt_adj, 1)
    adj_tilt_goal_delta = adj_tilt_goal - tilt_deg
    adj_tilt_dir = 1 if adj_tilt_goal_delta > 0 else -1

    if stab_tilt_enabled == True:
        if abs(adj_tilt_goal_delta) < pos_deg:
            stab_tilt_dps = 0.0
        else:
            stab_tilt_dps = round(
                stab_min_dps + (stab_max_dps - stab_min_dps) * min(1.0, abs(adj_tilt_goal_delta) / vel_deg), 1
            )

        dir_changed = stab_tilt_dps > 0 and adj_tilt_dir != last_tilt_dir
        dps_changed = abs(stab_tilt_dps - last_tilt_dps) > 0.5

        if dps_changed or dir_changed:
            last_time = stab_data_dict['last_tilt_vel_time']
            dt = start_time - last_time
            stab_data_dict['last_tilt_vel_time'] = start_time
            stab_data_dict['stab_tilt_vel_rate'] = round(float(1) / dt, 2) if dt > 0 else 0.0
            stab_data_dict['stab_tilt_dps'] = stab_tilt_dps
            stab_data_dict['stab_tilt_dir'] = adj_tilt_dir
            stab_data_dict['stab_tilt_goal'] = adj_tilt_goal

            if stab_tilt_dps == 0.0:
                sv_connect_if.stop_moving()
            else:
                sv_connect_if.jog_timed_speed_dps_tilt(adj_tilt_dir, stab_tilt_dps)

    return stab_data_dict, stab_settings_dict


PROCESSES_DICT['sv_stab_2'] = {'process_function': sv_stab_2, 
                                             'default_settings_dict': sv_stab_2_settings}



#########################
# Stab Utility Functions
#########################

def create_processes_dict():
    processes_dict = dict()
    for process_name in PROCESSES_DICT.keys():
        processes_dict[process_name] = PROCESSES_DICT[process_name]['default_settings_dict']
    return processes_dict

def update_processes_dict(stab_processes_dict):
    clean_stab_dict = create_processes_dict()
    for stab_process in clean_stab_dict.keys():
        if stab_process in stab_processes_dict.keys():
            for key in clean_stab_dict[stab_process].keys():
                if key in stab_processes_dict[stab_process].keys() and key != 'stab_controls_dict':
                    clean_stab_dict[stab_process][key] = stab_processes_dict[stab_process][key]
            for key in clean_stab_dict[stab_process]['stab_controls_dict'].keys():
                if key in stab_processes_dict[stab_process]['stab_controls_dict'].keys():
                    clean_stab_dict[stab_process]['stab_controls_dict'][key] = stab_processes_dict[stab_process]['stab_controls_dict'][key]
    return clean_stab_dict

def get_blank_data_dict():
    return copy.deepcopy(DATA_DICT)