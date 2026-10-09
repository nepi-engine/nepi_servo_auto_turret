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

import copy
import math

import numpy as np

from nepi_sdk import nepi_utils
from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_process
from nepi_sdk import nepi_controls
from nepi_sdk import nepi_nav

from nepi_interfaces.msg import DevicePTXStatus
from nepi_interfaces.msg import NavPose
from nepi_app_auto_turret.msg import AutoTurretAxis

from nepi_sdk.nepi_sdk import logger as Logger
log_name = "nepi_process_auto"
logger = Logger(log_name = log_name)


########################
## REQUIRED Process IF Utilities
DEFAULT_PROCESS_NAME = 'auto'
DEFAULT_PROCESS = 'auto_1'


RESULTS_PUB_MSG = None
RESULTS_PUB_TYPE = None
RESULTS_PUB_DICT = None
RESULTS_PUB_TOPIC = None


IMAGE_PUB_TOPIC = None

########################
## Process Utility Functions


BLANK_AXIS_DICT = nepi_sdk.convert_msg2dict(AutoTurretAxis)

def get_blank_axis_dict():
  axis_dict = copy.deepcopy(BLANK_AXIS_DICT)
  axis_dict['now_deg'] = 0
  axis_dict['lock_deg'] = 0
  axis_dict['lock_nav'] = 0
  axis_dict['goal_ratio'] = 0.5
  axis_dict['goal_deg'] = 0
  axis_dict['goal_nav'] = 0
  axis_dict['error_deg'] = 0
  axis_dict['speed_lock'] = 0
  axis_dict['speed_ratio'] = 0.5
  axis_dict['speed_dps'] = 0
  return axis_dict

BLANK_UPDATE_DICT = {
    'scanning': None,
    'tracking': None,
    'stabilize': None,
    'stop': None,
    'click_deg': None,
    'pos_ratio': None,
    'pos_deg': None,
    'pos_home': None,
    'speed_ratio': None,
    'speed_dps': None,
}

def get_blank_update_dict():
  return copy.deepcopy(BLANK_UPDATE_DICT)


BASE_RESULTS_DICT = dict()

def get_blank_results_dict():
  results_dict = copy.deepcopy(BASE_RESULTS_DICT)
  results_dict['timestamp'] = -999
  results_dict['pan_goal'] = -999
  results_dict['pan_error'] = -999
  results_dict['pan_speed'] = -999
  results_dict['tilt_goal'] = -999
  results_dict['tilt_error'] = -999
  results_dict['tilt_speed'] = -999
  results_dict['heading_goal'] = -999
  results_dict['heading_error'] = -999
  results_dict['roll_goal'] = -999
  results_dict['roll_error'] = -999
  results_dict['pitch_goal'] = -999
  results_dict['pitch_error'] = -999
  return results_dict


def update_results(results_dict, navpose_dict = None):
    if results_dict is not None:        
        timestamp = results_dict.get('timestamp',-999)
        if timestamp == -999:
            timestamp == nepi_utils.get_time()
        age_sec =  nepi_utils.get_time() - timestamp
        results_dict['timestamp'] = timestamp
        results_dict['age_sec'] = age_sec
        #logger.log_warn("Updated Results Dict: " + str([results_dict]), throttle_s = 5)
    return results_dict




########################
## Process Functions   
#######################
processes_dict = dict()
functions_dict = dict()


   
BASE_DATA_DICT = dict(
        scanning_enabled = False,
        tracking_enabled = False,
        stabilize_enabled = False,

        pan_update_dict = get_blank_update_dict(),
        tilt_update_dict = get_blank_update_dict(),

        pan_axis_dict = get_blank_axis_dict(),
        tilt_axis_dict = get_blank_axis_dict(),

        pantilt_dict = nepi_sdk.convert_msg2dict(DevicePTXStatus()),
        navpose_dict = nepi_sdk.convert_msg2dict(NavPose()),

        track_results_dict = None,
        scan_results_dict = None,
        stab_results_dict = None,

        last_results_time = 0,
    )


BASE_CONTROLS_DICT = dict(

        move_cont = {"type":"Toggle", "value":True,
                   # OPTIONAL
                   'display_name':'Move Continous', 'description':'Move pan tilt continous', 'display_hidden':False}, 

    )


BASE_RESULTS_DISPLAY_DICT = dict(


        pan_goal = {"type":"Float", "value":-999, 'round_value': 2,
                    # OPTIONAL
                    'display_name':'Pan (Deg)', 'description':'Pan Goal Degrees', 'display_hidden':False, 'round_display': 1,},

        tilt_goal = {"type":"Float", "value":-999, 'round_value': 2,
                    # OPTIONAL
                    'display_name':'Tilt (Deg)', 'description':'Tilt Goal Degrees', 'display_hidden':False, 'round_display': 1,},

        heading_goal = {"type":"Float", "value":-999, 'round_value': 2,
                    # OPTIONAL
                    'display_name':'Heading (Deg)', 'description':'Heading Goal Degrees', 'display_hidden':False, 'round_display': 1,},

        roll_goal = {"type":"Float", "value":-999, 'round_value': 2,
                    # OPTIONAL
                    'display_name':'Roll (Deg)', 'description':'Roll Goal Degrees', 'display_hidden':False, 'round_display': 1,},

        pitch_goal = {"type":"Float", "value":-999, 'round_value': 2,
                    # OPTIONAL
                    'display_name':'Pitch (Deg)', 'description':'Pitch Goal Degrees', 'display_hidden':False, 'round_display': 1,},



    )

BASE_STATES_DICT = dict(

    )


########################
## Process 1   



auto_1_dict = {

   
    'data_dict': copy.deepcopy(BASE_DATA_DICT),


    'controls_dict': copy.deepcopy(BASE_CONTROLS_DICT),


    'results_display_dict':  copy.deepcopy(BASE_RESULTS_DISPLAY_DICT),

    'states_dict':  copy.deepcopy(BASE_STATES_DICT),
}


def auto_1_process(data_dict, controls_dict, states_dict, results_dict, if_dict):
    start_time = nepi_utils.get_time()
    last_data_dict = copy.deepcopy(data_dict)
    last_results_dict = copy.deepcopy(results_dict)
    if last_results_dict is None:
        last_results_dict = get_blank_results_dict()
    controls_values_dict = nepi_controls.get_values_dict(controls_dict)
    #logger.log_warn("Got  Data: " + str(data_dict), throttle_s = 10)
    #logger.log_warn("Got  Data,Controls: " + str([data_dict,controls_values_dict]), throttle_s = 10)


    #logger.log_warn("Got Data and Controls: " + str([data_dict, controls_dict]), throttle_s = 5)

    ##############
    # Get Controls
    move_cont = controls_values_dict.get('move_cont', False)


    ##############
    # Get Data
    blank_data_dict = copy.deepcopy(BASE_DATA_DICT)

    scanning_enabled = data_dict.get('scanning_enabled',blank_data_dict['scanning_enabled'])
    tracking_enabled = data_dict.get('tracking_enabled',blank_data_dict['tracking_enabled'])
    stabilize_enabled = data_dict.get('stabilize_enabled',blank_data_dict['stabilize_enabled'])

    pan_update_dict = data_dict.get('pan_update_dict',blank_data_dict['pan_update_dict'])
    tilt_update_dict = data_dict.get('tilt_update_dict',blank_data_dict['tilt_update_dict'])

    pan_axis_dict = data_dict.get('pan_axis_dict',blank_data_dict['pan_axis_dict'])
    tilt_axis_dict = data_dict.get('tilt_axis_dict',blank_data_dict['tilt_axis_dict'])

    pantilt_dict = data_dict.get('pantilt_dict',blank_data_dict['pantilt_dict'])
    if pantilt_dict is None:
        pantilt_dict = nepi_sdk.convert_msg2dict(DevicePTXStatus())
        
    navpose_dict = data_dict.get('navpose_dict',blank_data_dict['navpose_dict'])
    if navpose_dict is None:
        navpose_dict = nepi_sdk.convert_msg2dict(NavPose())

    scan_results_dict = data_dict.get('scan_results_dict',None)
    track_results_dict = data_dict.get('track_results_dict',None)
    stab_results_dict = data_dict.get('stab_results_dict',None)

    pan_now = pantilt_dict['pan_now_deg']
    tilt_now = pantilt_dict['tilt_now_deg']

    pan_goal = pantilt_dict['pan_goal_deg']
    tilt_goal = pantilt_dict['tilt_goal_deg']

    if navpose_dict is None:
        navpose_dict = nepi_nav.BLANK_NAVPOSE_DICT
    if navpose_dict['has_pan_tilt'] == True:
        if navpose_dict['has_heading'] == True:
            heading_deg = navpose_dict.get('pan_tilt_heading_deg',-999)
        else:
            heading_deg = navpose_dict.get('pan_tilt_yaw_deg',-999)
        pitch_deg = navpose_dict.get('pan_tilt_pitch_deg',-999) 
        roll_deg = navpose_dict.get('pan_tilt_roll_deg',-999)
    else:
        if navpose_dict['has_heading'] == True:
            heading_deg = navpose_dict.get('heading_deg',-999)
        else:
            heading_deg = navpose_dict.get('yaw_deg',-999)
        pitch_deg = navpose_dict.get('pitch_deg',-999) 
        roll_deg = navpose_dict.get('roll_deg',-999)

    if int(heading_deg) == -999:
        heading_deg = 0
    
    if int(pitch_deg) == -999:
        pitch_deg = 0
      
    if int(roll_deg) == -999:
        roll_deg = 0



    ##############
    # Run Process
    #############

    if tracking_enabled == True and track_results_dict is not None:
      
      pan_pos_disabled  = True
      pan_jog_disabled = True
      pan_speed_disabled = True

      tilt_pos_disabled  = True
      tilt_jog_disabled = True
      tilt_speed_disabled = True

      track_heading_deg = track_results_dict['heading_deg']
      track_pitch_deg = track_results_dict['pitch_deg']
      track_roll_deg = track_results_dict['roll_deg']

      auto_pan_error = -1 * (heading_deg - track_heading_deg)
      auto_tilt_error = -1 * (pitch_deg - track_pitch_deg)

    else:
      pan_pos_disabled = False
      pan_jog_disabled = False
      pan_speed_disabled = False

      tilt_pos_disabled  = False
      tilt_jog_disabled = False
      tilt_speed_disabled = False
      
      auto_pan_error = -1 * (pan_now - pan_goal)
      auto_tilt_error = -1 * (tilt_now - tilt_goal)

    #######
    # Process Data
    auto_pan_goal = round(pan_now + auto_pan_error,1)
    auto_pan_error = round(auto_pan_error,1)

    auto_tilt_goal = round(tilt_now + auto_tilt_error,1)
    auto_tilt_error = round(auto_tilt_error,1)

    # auto_heading_goal = round(heading_deg + auto_pan_error, 1) 
    # auto_heading_error = round(auto_pan_error,1)

    ##############
    # Process Results
    #############
    results_dict = get_blank_results_dict()

    results_dict['pan_goal'] = auto_pan_goal
    results_dict['pan_error'] = auto_pan_error
    results_dict['pan_dps'] = -999

    results_dict['tilt_goal'] = auto_tilt_goal
    results_dict['tilt_error'] = auto_tilt_error
    results_dict['tilt_dps'] = -999

    results_dict['heading_goal'] = -999
    results_dict['heading_error'] = -999
    results_dict['roll_goal'] = -999
    results_dict['roll_error'] = -999
    results_dict['pitch_goal'] = -999
    results_dict['pitch_error'] = -999

    results_dict = update_results(results_dict, navpose_dict)

    ###########################
    # Update PanTilt Axis Data

    ###############
    # Pan
    pan_axis_dict['pos_disabled'] = pan_pos_disabled
    pan_axis_dict['jog_disabled'] = pan_jog_disabled
    pan_axis_dict['speed_disabled'] = pan_speed_disabled

    pan_axis_dict['now_deg'] = pan_now
    # pan_axis_dict['lock_deg'] = pan_lock_deg

    # pan_axis_dict['goal_ratio'] = pan_goal_ratio
    pan_axis_dict['goal_deg'] = auto_pan_goal
    pan_axis_dict['error_deg'] = auto_pan_error

    # pan_axis_dict['speed_lock'] = pan_speed_lock
    # pan_axis_dict['speed_ratio'] = pan_speed_ratio
    # pan_axis_dict['speed_dps'] = pan_speed_dps

    # ###############
    # # Tilt

    tilt_axis_dict['pos_disabled'] = tilt_pos_disabled
    tilt_axis_dict['jog_disabled'] = tilt_jog_disabled
    tilt_axis_dict['speed_disabled'] = tilt_speed_disabled

    tilt_axis_dict['now_deg'] = tilt_now
    # tilt_axis_dict['lock_deg'] = tilt_lock_deg

    # tilt_axis_dict['goal_ratio'] = tilt_goal_ratio
    tilt_axis_dict['goal_deg'] = auto_tilt_goal
    tilt_axis_dict['error_deg'] = auto_tilt_error

    # tilt_axis_dict['speed_lock'] = tilt_speed_lock
    # tilt_axis_dict['speed_ratio'] = tilt_speed_ratio
    # tilt_axis_dict['speed_dps'] = tilt_speed_dps


    # ###########################
    # # Apply PanTilt Control Updates
    # pantilt_connect_if = if_dict.get('pantilt_connect_if', None)
    # if pantilt_connect_if is not None:
    #     if pantilt_connect_if.get_ready() == True:
    #         pass
    
    #logger.log_warn("Process Completed: " + str([auto_dict,results_dict,len(filtered_targets),len(targets_dict_list),class_filters]), throttle_s = 5)
    return data_dict, controls_dict, states_dict, results_dict


processes_dict = nepi_process.update_processes_dict(processes_dict, process_name = 'auto_1', process_dict = auto_1_dict)
#logger.log_warn("Updated processes dict: " + str(processes_dict))
functions_dict['auto_1'] = auto_1_process




########################
## Processes Init Dict  
PROCESSES_DICT = copy.deepcopy(processes_dict)
FUNCTIONS_DICT = functions_dict



########################
## Process Image Functions   
#######################

