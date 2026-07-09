#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi applications (nepi_apps) repo
# (see https://https://github.com/nepi-engine/nepi_apps)
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
#
import os
import time
import sys
import numpy as np
import copy
import math
import threading
import importlib


from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_targets
from nepi_sdk import nepi_track
from nepi_sdk import nepi_nav


from nepi_sdk import nepi_stab_sv

from nepi_app_servo_auto.msg import ServoAutoAppStatus
from nepi_interfaces.msg import DeviceSVXStatus, RangeWindow, ImageMouseEvent, MgrSystemStatus
from nepi_interfaces.msg import RangeWindow, Target, Targets, TargetingStatus, TargetingUpdate
from nepi_interfaces.msg import Track, TrackingStatus
from nepi_interfaces.msg import NavPose, NavPoseOrientation
from nepi_interfaces.msg import Predict, PredictStatus, PredictProcess
from nepi_interfaces.msg import UpdateBool, UpdateFloat


from std_msgs.msg import UInt8, Int32, Float32, Empty, String, Bool, Header

from nepi_api.node_if import NodeClassIF
from nepi_api.messages_if import MsgIF
from nepi_api.connect_device_if_svx import ConnectSVXDeviceIF


UPDATE_IMAGE_SUBS_RATE_HZ = 1
UPDATE_SAVE_DATA_CHECK_RATE_HZ = 10

TARGET_TOPIC_TIMEOUT_SEC = 2
TARGET_TRACK_TIMEOUT_SEC = 2

#########################################
# Node Class
#########################################

class NepiServoAutoApp(object):

  PAN_MIN_MAX_DEG = 165
  TILT_MIN_MAX_DEG = 50

  SCAN_SWITCH_DEG = 5 # If angle withing this bound, switch dir
  SCAN_UPDATE_INTERVAL = .5

  MIN_SCAN_ANGLE = 30
  LIMIT_PADDING = 5

  TRACK_UPDATE_INTERVAL = 0.2
  TRACK_MOVE_DEG = 2
  TRACK_GOAL_DEG = 10

  TRACK_DEFAULT_SOURCE = 'targets'
  TRACK_MOVE_RATIO = 0.8
  TRACK_MAX_RESET_SEC = 5
  TRACK_RESET_TIME_SEC = 2
  TRACK_DEFAULT_TARGETS = ['person']

  TARGET_BEST_FILTER_OPTIONS = nepi_track.BEST_FILTER_OPTIONS
  TARGET_BEST_FILTER_DEFAULT = 'LARGEST'

  STAB_DEFAULT_SOURCE = 'microstrain'

  IMAGE_PRIORITY_OPTIONS = ['IMAGES','DETECTIONS','TARGETS']
  IMAGE_PRIORITY_NAMES = ['color_image','detection_image','target_image']



  #####################
  
  node_if = None
  process_needs_update = False
  status_msg = ServoAutoAppStatus() 
  status_has_published = False
  status_update_rate = 1

  #####################

  available_servos = []
  selected_servo = "None"
  sv_connect_if = None

  sv_connected_topic = None
  sv_connected = False

  servo_max_speed_dps = -999
  servo_avg_move_delay = 0.2
  pan_deg_per_sec = -999
  tilt_deg_per_sec = -999

  speed_ratio = 1.0
  pan_speed_ratio = 1.0
  tilt_speed_ratio = 1.0

  min_pan_softstop_deg = -PAN_MIN_MAX_DEG
  max_pan_softstop_deg = PAN_MIN_MAX_DEG
  min_tilt_softstop_deg = -TILT_MIN_MAX_DEG
  max_tilt_softstop_deg = TILT_MIN_MAX_DEG

  goto_position = [0,0]

  navpose_update_rate = 1
  status_update_rate = 1

  current_position = None

  #####################
  has_scan_pan = True
  has_scan_tilt = True
  has_sin_pan = False
  has_sin_tilt = False
  has_homing = False
  has_set_home = False

  pan_scanning = False
  tilt_scanning = False

  pan_tracking = False
  tilt_tracking = False

  pan_stabing = False
  tilt_stabing = False

  #####################

  scan_pan_enabled = False
  pan_track_hold = False

  scan_pan_sec = 5

  scan_pan_last_time = None
  scan_pan_times = [0,0,0,0,0]
  scan_pan_time = 1
  sin_pan_enabled = False
  scan_pan_sin_ind = 0
  
  scan_tilt_enabled = False
  tilt_track_hold = False
  scan_tilt_sec = 5

  scan_tilt_last_time = None
  scan_tilt_times = [0,0,0,0,0]
  scan_tilt_time = 1
  sin_tilt_enabled = False
  scan_tilt_sin_ind = 0

  scan_pan_speed_ratio = 1.0
  scan_tilt_speed_ratio = 1.0

  scan_pan_min_deg = -PAN_MIN_MAX_DEG
  scan_pan_max_deg = PAN_MIN_MAX_DEG
  scan_tilt_min_deg = -TILT_MIN_MAX_DEG
  scan_tilt_max_deg = TILT_MIN_MAX_DEG


  scan_tilt_sins = 0
  scan_tilt_sin_ind = 0
  scan_tilt_sins = 0
  scan_tilt_sin_ind = 0

  #####################
  targets_msg = None
  targets_last_time = 0
  

  last_track_pan_time = 0
  last_track_tilt_time = 0


  track_pan_enabled = False
  track_tilt_enabled = False


  track_range_min_m = 0
  track_range_max_m = 1000
  track_pan_min_deg = -PAN_MIN_MAX_DEG
  track_pan_max_deg = PAN_MIN_MAX_DEG
  track_tilt_min_deg = -TILT_MIN_MAX_DEG
  track_tilt_max_deg = TILT_MIN_MAX_DEG


  track_move_deg = TRACK_MOVE_DEG
  track_goal_deg = TRACK_GOAL_DEG
  track_reset_time_sec = TRACK_RESET_TIME_SEC
  track_move_ratio = TRACK_MOVE_RATIO

  track_pan_dict = None
  track_tilt_dict = None

  track_num_avg = 1
  track_pan_error = 0
  track_tilt_error = 0

  track_if = None


  #####################


  navpose_msg = None
  last_navpose_time = 0

  stab_timeout = 2
  stab_count = 0

  stab_pan_ready = False
  stab_tilt_ready = False

  stab_pan_enabled = False
  stab_tilt_enabled = False

  stab_position = [0,0]
  
  available_stab_source_dict = dict()
  selected_stab_source = 'None'
  stab_source_connected_namespace = "None"
  stab_source_connecting = False
  stab_source_connected = False
  last_stab_source_time = None
  stab_source_dict = None
  stab_source_lock = threading.Lock()

  stab_subpub_dict = None
  stab_subpub_lock = threading.Lock()

  available_stab_processes = list(nepi_stab_sv.PROCESSES_DICT.keys())
  stab_processes_dict = nepi_stab_sv.create_processes_dict()
  selected_stab_process = nepi_stab_sv.DEFAULT_PROCESS
  stab_process_ready = True


  stab_data_dict = nepi_stab_sv.get_blank_data_dict()
  stab_data_dict_last = None
  stab_dict_lock = threading.Lock()
  stab_pan_speed_start = 1.0
  stab_tilt_speed_start = 1.0
  stab_pan_adj = 0.0
  stab_tilt_adj = 0.0


  ############

  click_pan_enabled = True
  click_tilt_enabled = True
  set_mouse_click = [0,0]

  active_nodes = []
  active_topics = []
  active_topic_types = []
  active_services = []

  #################
  ### Image Viewer
  #################
  FACTORY_SELECTED_PAN_TILTS = ["None","None","None","None"]

  update_image_subs_interval_sec = float(1)/UPDATE_IMAGE_SUBS_RATE_HZ
 
  single_image_topic = "None"
  selected_image_topics = ["None","None","None","None"]
  num_windows = 1
  last_num_windows = 4

  image_priority_list = []
  image_priority_dict = dict()
  for i, option in enumerate(IMAGE_PRIORITY_OPTIONS):
     image_priority_dict[option] = IMAGE_PRIORITY_NAMES[i]

  available_image_topics = []  
  available_image_dict = dict()



  ###############
  # Tracking
  ###############

  targeting_topic = 'targets'
  targets_status_msg = None
  targets_status_msg_start = None
  targets_status_last_time = None
  targets_timeout = 3
  targets_last_time = 0


  tracking_topic = 'track'
  tracking_status_msg = TrackingStatus()

  tracking_enabled = True
  tracking_running = False
  tracking_state = False

  tracking_manages_targeting = False

  
  tracking_info_dict = None
  tracking_subpub_dict = None
  tracking_subpub_lock = threading.Lock()

  tracking_targets_topic = 'None'
  tracking_targets_connecting = False
  tracking_targets_connected = False
  tracking_targets_connected_namespace = "None"
  tracking_last_targets = 'None'



  tracking_available_targets = []
  tracking_available_sources = []
  tracking_available_classes = []
  tracking_best_filter_options = copy.deepcopy(nepi_track.BEST_FILTER_OPTIONS)

  tracking_dict = copy.deepcopy(nepi_track.BLANK_SETTINGS_DICT)

  track_dict = None
  track_dict_check = None
  track_timeout = 3
  track_last_time = 0



  ###############
  # Predict
  ###############

  # PREDICT_MIN_LOG_TIME = 1
  # PREDICT_MAX_LOG_TIME = 100
  # PREDICT_DEFAULT_LOG_TIME = 5

  # PREDICT_MIN_LOG_RATE = 1
  # PREDICT_MAX_LOG_RATE = 10
  # PREDICT_DEFAULT_LOG_RATE = 2

  # PREDICT_MIN_PREDICT_TIME = 0.1
  # PREDICT_MAX_PREDICT_TIME = 1
  # PREDICT_DEFAULT_PREDICT_TIME = 1

  # PREDICT_DEFAULT_QUALITY_FILTER = 0.5

  # predict_topic = 'predict'
  # predict_status_msg = PredictStatus()


  # predict_settings_dict = nepi_predict.create_predict_dict(predict_data_names)
  # predict_settings_dict = nepi_predict.update_predict_dict('max_log_time', PREDICT_DEFAULT_LOG_TIME, predict_settings_dict)
  # predict_settings_dict = nepi_predict.update_predict_dict('max_log_rate', PREDICT_DEFAULT_LOG_RATE, predict_settings_dict)
  # predict_settings_dict = nepi_predict.update_predict_dict('predict_time', PREDICT_DEFAULT_PREDICT_TIME, predict_settings_dict)
  # predict_settings_dict = nepi_predict.update_predict_dict('quality_filter', PREDICT_DEFAULT_QUALITY_FILTER, predict_settings_dict)
  
  # predict_data_dict = nepi_predict.create_datas_dict(predict_settings_dict)
  # predict_data_lock = threading.Lock()

  
  #######################
  ### Node Initialization
  DEFAULT_NODE_NAME = "app_servo_scan" # Can be overwitten by luanch command
  def __init__(self):
    #### APP NODE INIT SETUP ####
    nepi_sdk.init_node(name= self.DEFAULT_NODE_NAME)
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


    self.scan_pan_times = [0,0,0,0,0]
    self.scan_tilt_times = [0,0,0,0,0]
    self.scan_pan_sins = []
    self.scan_pan_sin_ind = 0
    self.scan_tilt_sins = []
    self.scan_tilt_sin_ind = 0


    
    # SCAN SCANNING ##############
    # timed scan scanning is not supported yet


    ##############################
    ### Setup Node

    # Configs Config Dict ####################
    self.CFGS_DICT = {
            'init_callback': self.initCb,
            'reset_callback': self.resetCb,
            'factory_reset_callback': self.factoryResetCb,
            'init_configs': True,
            'namespace': self.node_namespace      

    }

    # Params Config Dict ####################
    self.PARAMS_DICT = {

        #####################
        ###Servo
        #####################
        'selected_servo': {
            'namespace': self.node_namespace,
            'factory_val': self.selected_servo
        },
        'speed_ratio': {
            'namespace': self.node_namespace,
            'factory_val': self.speed_ratio
        },
        'pan_speed_ratio': {
            'namespace': self.node_namespace,
            'factory_val': self.pan_speed_ratio
        },
        'tilt_speed_ratio': {
            'namespace': self.node_namespace,
            'factory_val': self.tilt_speed_ratio
        },



        #####################
        ###Scan
        #####################

        'scan_pan_enabled': {
            'namespace': self.node_namespace,
            'factory_val': False
        },          
        'scan_tilt_enabled': {
            'namespace': self.node_namespace,
            'factory_val': False
        },      
        'scan_pan_min_deg': {
            'namespace': self.node_namespace,
            'factory_val': self.scan_pan_min_deg
        },           
        'scan_pan_max_deg': {
            'namespace': self.node_namespace,
            'factory_val':self.scan_pan_max_deg
        },   
        'scan_tilt_min_deg': {
            'namespace': self.node_namespace,
            'factory_val': self.scan_tilt_min_deg
        },           
        'scan_tilt_max_deg': {
            'namespace': self.node_namespace,
            'factory_val':self.scan_tilt_max_deg
        },    
        'scan_pan_speed_ratio': {
            'namespace': self.node_namespace,
            'factory_val': self.scan_pan_speed_ratio
        },
        'scan_tilt_speed_ratio': {
            'namespace': self.node_namespace,
            'factory_val': self.scan_tilt_speed_ratio
        },
        'sin_pan_enabled': {
            'namespace': self.node_namespace,
            'factory_val': False
        },
        'sin_tilt_enabled': {
            'namespace': self.node_namespace,
            'factory_val': False
        },

        #####################
        ###Tracking
        #####################
        'tracking_manages_targeting': {
            'namespace': self.node_namespace,
            'factory_val': self.tracking_manages_targeting
        },
        'tracking_dict': {
            'namespace': self.node_namespace,
            'factory_val': self.tracking_dict
        },
        'track_pan_enabled': {
            'namespace': self.node_namespace,
            'factory_val': False
        },  
        'track_tilt_enabled': {
            'namespace': self.node_namespace,
            'factory_val': False
        },   
        'track_pan_min_deg': {
            'namespace': self.node_namespace,
            'factory_val': self.track_pan_min_deg
        },           
        'track_pan_max_deg': {
            'namespace': self.node_namespace,
            'factory_val':self.track_pan_max_deg
        },   
        'track_tilt_min_deg': {
            'namespace': self.node_namespace,
            'factory_val': self.track_tilt_min_deg
        },           
        'track_tilt_max_deg': {
            'namespace': self.node_namespace,
            'factory_val':self.track_tilt_max_deg
        },          
        'track_reset_time_sec': {
            'namespace': self.node_namespace,
            'factory_val': self.track_reset_time_sec
        },      

        'track_move_ratio': {
            'namespace': self.node_namespace,
            'factory_val': self.track_move_ratio
        },
        'track_goal_deg': {
            'namespace': self.node_namespace,
            'factory_val': self.track_goal_deg
        },
        'track_move_deg': {
            'namespace': self.node_namespace,
            'factory_val': self.track_move_deg
        },

        #####################
        ###Stab
        #####################

        'selected_stab_source': {
            'namespace': self.node_namespace,
            'factory_val': self.selected_stab_source
        },
        'stab_processes_dict': {
            'namespace': self.node_namespace,
            'factory_val': self.stab_processes_dict
        },
        'selected_stab_process': {
            'namespace': self.node_namespace,
            'factory_val': self.selected_stab_process
        },
        'stab_pan_enabled': {
            'namespace': self.node_namespace,
            'factory_val': False
        },  
        'stab_tilt_enabled': {
            'namespace': self.node_namespace,
            'factory_val': False
        },
 


        #####################
        ###Image Viewer
        #####################

        'single_image_topic': {
            'namespace': self.node_namespace,
            'factory_val': self.single_image_topic
        },
        'selected_image_topics': {
            'namespace': self.node_namespace,
            'factory_val': self.selected_image_topics
        },
        'num_windows': {
            'namespace': self.node_namespace,
            'factory_val': self.num_windows
        },

 



    }

    # Publishers Config Dict ####################
    self.PUBS_DICT = {
        'status_pub': {
            'namespace': self.node_namespace,
            'topic': 'status',
            'msg': ServoAutoAppStatus,
            'qsize': 1,
            'latch': True
        },
        #####################
        ### Tracking
        #####################
        'track': {
            'msg': Target,
            'namespace': self.node_namespace,
            'topic': self.tracking_topic,
            'qsize': 1,
            'latch': True
        },
        'track_status': {
            'msg': TrackingStatus,
            'namespace': self.node_namespace + '/' + self.tracking_topic,
            'topic': 'status',
            'qsize': 1,
            'latch': True
        }
    }

    # Subscribers Config Dict ####################
    self.SUBS_DICT = {

        #####################
        ### Servo
        #####################
        'select_pan_and_tilt': {
            'namespace': self.node_namespace,
            'topic': 'select_sv_device',
            'msg': String,
            'qsize': None,
            'callback': self.selectTopicCb, 
            'callback_args': ()
        },
        'set_speed_ratio': {
            'namespace': self.node_namespace,
            'topic': 'set_speed_ratio',
            'msg': Float32,
            'qsize': 1,
            'callback': self.setSpeedRatioCb,
            'callback_args': ()
        },
        'set_pan_speed_ratio': {
            'namespace': self.node_namespace,
            'topic': 'set_pan_speed_ratio',
            'msg': Float32,
            'qsize': 1,
            'callback': self.setPanSpeedRatioCb,
            'callback_args': ()
        },
        'set_tilt_speed_ratio': {
            'namespace': self.node_namespace,
            'topic': 'set_tilt_speed_ratio',
            'msg': Float32,
            'qsize': 1,
            'callback': self.setTiltSpeedRatioCb,
            'callback_args': ()
        },
        'servo_home': {
            'namespace': self.node_namespace,
            'topic': 'servo_home',
            'msg': Empty,
            'qsize': 1,
            'callback': self.servoHomeCb,
            'callback_args': ()
        },
        'pan_home': {
            'namespace': self.node_namespace,
            'topic': 'pan_home',
            'msg': Empty,
            'qsize': 1,
            'callback': self.panHomeCb,
            'callback_args': ()
        },
        'tilt_home': {
            'namespace': self.node_namespace,
            'topic': 'tilt_home',
            'msg': Empty,
            'qsize': 1,
            'callback': self.tiltHomeCb,
            'callback_args': ()
        },

        #####################
        ###Scan
        #####################

        'set_scan_pan': {
            'namespace': self.node_namespace,
            'topic': 'set_scan_pan_enable',
            'msg': Bool,
            'qsize': 1,
            'callback': self.setScanPanCb, 
            'callback_args': ()
        },
        'set_scan_tilt_enable': {
            'namespace': self.node_namespace,
            'topic': 'set_scan_tilt_enable',
            'msg': Bool,
            'qsize': 1,
            'callback': self.setScanTiltCb, 
            'callback_args': ()
        },
        'set_scan_pan_window': {
            'namespace': self.node_namespace,
            'topic': 'set_scan_pan_window',
            'msg': RangeWindow,
            'qsize': 1,
            'callback': self.setScanPanWindowCb, 
            'callback_args': ()
        },
        'set_scan_tilt_window': {
            'namespace': self.node_namespace,
            'topic': 'set_scan_tilt_window',
            'msg': RangeWindow,
            'qsize': 1,
            'callback': self.setScanTiltWindowCb, 
            'callback_args': ()
        },
        'set_scan_pan_speed_ratio': {
            'namespace': self.node_namespace,
            'topic': 'set_scan_pan_speed_ratio',
            'msg': Float32,
            'qsize': 1,
            'callback': self.setScanPanSpeedRatioCb,
            'callback_args': ()
        },
        'set_scan_tilt_speed_ratio': {
            'namespace': self.node_namespace,
            'topic': 'set_scan_tilt_speed_ratio',
            'msg': Float32,
            'qsize': 1,
            'callback': self.setScanTiltSpeedRatioCb,
            'callback_args': ()
        },

        #####################
        ###Tracking
        #####################
        'set_tracking_manages_targeting': {
            'namespace': self.node_namespace + '/' + self.tracking_topic,
            'topic': 'set_tracking_manages_targeting',
            'msg': Bool,
            'qsize': 10,
            'callback': self.setTrackManagesTargetingCb, 
            'callback_args': ()
        },
        'set_tracking_targets_topic': {
            'namespace': self.node_namespace + '/' + self.tracking_topic,
            'topic': 'set_targets_topic',
            'msg': String,
            'qsize': 10,
            'callback': self.setTrackTargetsTopicCb, 
            'callback_args': ()
        },
        'set_tracking_source_topic': {
            'namespace': self.node_namespace + '/' + self.tracking_topic,
            'topic': 'set_source_topic',
            'msg': String,
            'qsize': 10,
            'callback': self.setTrackSourceTopicCb, 
            'callback_args': ()
        },
        'set_tracking_class_filter': {
            'namespace': self.node_namespace + '/' + self.tracking_topic,
            'topic': 'set_class_filter',
            'msg': String,
            'qsize': 10,
            'callback': self.setTrackClassFilterCb, 
            'callback_args': ()
        },
        'set_tracking_threshold_filter': {
            'namespace': self.node_namespace + '/' + self.tracking_topic,
            'topic': 'set_threshold_filter',
            'msg': Float32,
            'qsize': 10,
            'callback': self.setTrackThresholdFilterCb, 
            'callback_args': ()
        },
        'set_tracking_best_filter': {
            'namespace': self.node_namespace + '/' + self.tracking_topic,
            'topic': 'set_best_filter',
            'msg': String,
            'qsize': 10,
            'callback': self.setTrackBestFilterCb, 
            'callback_args': ()
        },


        #####################
        ###Track
        #####################
        'set_track_pan_enable': {
            'namespace': self.node_namespace,
            'topic': 'set_track_pan_enable',
            'msg': Bool,
            'qsize': 1,
            'callback': self.setTrackPanCb, 
            'callback_args': ()
        },
        'set_track_tilt_enable': {
            'namespace': self.node_namespace,
            'topic': 'set_track_tilt_enable',
            'msg': Bool,
            'qsize': 1,
            'callback': self.setTrackTiltCb, 
            'callback_args': ()
        },
        'set_track_pan_window': {
            'namespace': self.node_namespace,
            'topic': 'set_track_pan_window',
            'msg': RangeWindow,
            'qsize': 1,
            'callback': self.setTrackPanWindowCb, 
            'callback_args': ()
        },
        'set_track_tilt_window': {
            'namespace': self.node_namespace,
            'topic': 'set_track_tilt_window',
            'msg': RangeWindow,
            'qsize': 1,
            'callback': self.setTrackTiltWindowCb, 
            'callback_args': ()
        },
        'set_track_reset_time_sec': {
            'namespace': self.node_namespace,
            'topic': 'set_track_reset_time_sec',
            'msg': Float32,
            'qsize': 1,
            'callback': self.setTrackResetTimeSecCb, 
            'callback_args': ()
        },
        'set_track_move_ratio': {
            'namespace': self.node_namespace,
            'topic': 'set_track_move_ratio',
            'msg': Float32,
            'qsize': 1,
            'callback': self.setTrackMoveRatioCb,
            'callback_args': ()
        },
        'set_track_goal_deg': {
            'namespace': self.node_namespace,
            'topic': 'set_track_goal_deg',
            'msg': Float32,
            'qsize': 1,
            'callback': self.setTrackGoalDegCb,
            'callback_args': ()
        },
        'set_track_move_deg': {
            'namespace': self.node_namespace,
            'topic': 'set_track_move_deg',
            'msg': Float32,
            'qsize': 1,
            'callback': self.setTrackMoveDegCb,
            'callback_args': ()
        },



        #####################
        ###Stab
        #####################
        'reload_stabs': {
            'namespace': self.node_namespace,
            'topic': 'reload_stab_processes',
            'msg': Empty,
            'qsize': 10,
            'callback': self.reloadStabsCb, 
            'callback_args': ()
        },
         'set_stab_source': {
            'namespace': self.node_namespace,
            'topic': 'set_stab_source',
            'msg': String,
            'qsize': 10,
            'callback': self.setStabSourceCb, 
            'callback_args': ()
        },       

        'set_stab_process': {
            'namespace': self.node_namespace,
            'topic': 'set_stab_process',
            'msg': String,
            'qsize': 10,
            'callback': self.setStabProcessCb, 
            'callback_args': ()
        },
        'set_stab_pan_enable': {
            'namespace': self.node_namespace,
            'topic': 'set_stab_pan_enable',
            'msg': Bool,
            'qsize': 1,
            'callback': self.setStabPanCb, 
            'callback_args': ()
        },
        'set_stab_tilt_enable': {
            'namespace': self.node_namespace,
            'topic': 'set_stab_tilt_enable',
            'msg': Bool,
            'qsize': 1,
            'callback': self.setStabTiltCb, 
            'callback_args': ()
        },
        'set_stab_reset_time_sec': {
            'namespace': self.node_namespace,
            'topic': 'set_stab_reset_time_sec',
            'msg': Float32,
            'qsize': 1,
            'callback': self.setStabResetTimeSecCb, 
            'callback_args': ()
        },
        'set_stab_max_speed_ratios': {
            'namespace': self.node_namespace,
            'topic': 'set_stab_max_speed_ratios',
            'msg': RangeWindow,
            'qsize': 1,
            'callback': self.setStabSpeedRatiosCb,
            'callback_args': ()
        },
        'set_stab_update_rate': {
            'namespace': self.node_namespace,
            'topic': 'set_stab_update_rate',
            'msg': Float32,
            'qsize': 1,
            'callback': self.setStabUpdateRateCb,
            'callback_args': ()
        },
        'set_stab_num_avg': {
            'namespace': self.node_namespace,
            'topic': 'set_stab_num_avg',
            'msg': Int32,
            'qsize': 1,
            'callback': self.setStabNumAvgCb,
            'callback_args': ()
        },
        'set_stab_control_value': {
            'namespace': self.node_namespace,
            'topic': 'set_stab_control_value',
            'msg': UpdateFloat,
            'qsize': 1,
            'callback': self.setStabControlCb,
            'callback_args': ()
        },
        'set_stab_pan_pos_ratio': {
            'namespace': self.node_namespace,
            'topic': 'set_stab_pan_pos_ratio',
            'msg': Float32,
            'qsize': 1,
            'callback': self.setStabPanPosRatioCb,
            'callback_args': ()
        },
        'set_stab_tilt_pos_ratio': {
            'namespace': self.node_namespace,
            'topic': 'set_stab_tilt_pos_ratio',
            'msg': Float32,
            'qsize': 1,
            'callback': self.setStabTiltPosRatioCb,
            'callback_args': ()
        },


        #####################
        ###Misc
        #####################


        'set_pan_click_enable': {
            'namespace': self.node_namespace,
            'topic': 'set_pan_click_enable',
            'msg': Bool,
            'qsize': None,
            'callback': self.setPanClickCb, 
            'callback_args': ()
        },
        'set_tilt_click_enable': {
            'namespace': self.node_namespace,
            'topic': 'set_tilt_click_enable',
            'msg': Bool,
            'qsize': None,
            'callback': self.setTiltClickCb, 
            'callback_args': ()
        },
        'system_status': {
            'msg': MgrSystemStatus,
            'namespace': self.base_namespace,
            'topic': 'status',
            'qsize': 5,
            'callback': self.systemStatusCb
        },


        ######################
        ###Image Viewer
        ######################
        'set_mouse_click': {
            'namespace': self.node_namespace,
            'topic': 'set_mouse_click',
            'msg': ImageMouseEvent,
            'qsize': None,
            'callback': self.mouseClickCb, 
            'callback_args': ()
        },
        'set_topic_1': {
            'namespace': self.node_namespace,
            'topic': 'set_topic_1',
            'msg': String,
            'qsize': 10,
            'callback': self.setImageTopic1Cb, 
            'callback_args': ()
        },
        'set_topic_2': {
            'namespace': self.node_namespace,
            'topic': 'set_topic_2',
            'msg': String,
            'qsize': 10,
            'callback': self.setImageTopic2Cb, 
            'callback_args': ()
        },
          'set_topic_3': {
            'namespace': self.node_namespace,
            'topic': 'set_topic_3',
            'msg': String,
            'qsize': 10,
            'callback': self.setImageTopic3Cb, 
            'callback_args': ()
        },
          'set_topic_4': {
            'namespace': self.node_namespace,
            'topic': 'set_topic_4',
            'msg': String,
            'qsize': 10,
            'callback': self.setImageTopic4Cb, 
            'callback_args': ()
        },
          'set_num_windows': {
            'namespace': self.node_namespace,
            'topic': 'set_num_windows',
            'msg': Int32,
            'qsize': 10,
            'callback': self.setNumWindowsCb, 
            'callback_args': ()
        },
          'set_image_priority': {
            'namespace': self.node_namespace,
            'topic': 'set_image_priority',
            'msg': String,
            'qsize': 10,
            'callback': self.setImagePriorityCb, 
            'callback_args': ()
        },
           
    }


    # Create Node Class ####################
    self.node_if = NodeClassIF(
                    configs_dict = self.CFGS_DICT,
                    params_dict = self.PARAMS_DICT,
                    pubs_dict = self.PUBS_DICT,
                    subs_dict = self.SUBS_DICT
    )

    nepi_sdk.wait()
    


    ##############################
    self.initCb(do_updates = True)

    ##############################
    # Start updater process
    nepi_sdk.start_timer_process(1.0, self.updaterCb, oneshot = True)
    nepi_sdk.start_timer_process(0.2, self.updaterServoCb, oneshot = True)

    self.msg_if.pub_warn("Starting status pub")
    nepi_sdk.start_timer_process(0.5, self.publishStatusCb)

    self.msg_if.pub_warn("has_scan_pan: " + str(self.has_scan_pan))
    if self.has_scan_pan:
        # Start Scan Pan Process
        self.msg_if.pub_info("Starting scan pan scanning process")
        nepi_sdk.start_timer_process(self.SCAN_UPDATE_INTERVAL, self.scanPanProcess)
        nepi_sdk.start_timer_process(self.TRACK_UPDATE_INTERVAL, self.trackPanProcess, oneshot = True)


    if self.has_scan_tilt:
        # Start Scan Pan Process
        self.msg_if.pub_info("Starting scan tilt scanning process")
        nepi_sdk.start_timer_process(self.SCAN_UPDATE_INTERVAL, self.scanTiltProcess)
        nepi_sdk.start_timer_process(self.TRACK_UPDATE_INTERVAL, self.trackTiltProcess, oneshot = True)



    #self.msg_if.pub_warn("supports_sin_scan: " + str(self.supports_sin_scan))
    #if self.supports_sin_scan:
        #self.msg_if.pub_warn("Starting sin scanning process")
        #nepi_sdk.start_timer_process(.5, self.scanPanSinProcess, oneshot = True)
        #nepi_sdk.start_timer_process(.5, self.scanTiltSinProcess, oneshot = True)


    nepi_sdk.start_timer_process(1.0, self.updaterTrackingCb, oneshot = True)
    nepi_sdk.start_timer_process(1.0, self.updaterStabCb, oneshot = True)
    nepi_sdk.start_timer_process(1.0, self.updaterStabSolutionCb, oneshot = True)  
    nepi_sdk.start_timer_process(1.0, self.updaterTrackingStateCb, oneshot = True)




    ##############################
    ## Initiation Complete
    self.msg_if.pub_info(" Initialization Complete")

    # Spin forever (until object is detected)
    nepi_sdk.spin()
    ##############################

#######################
  ### App Config Functions

  ####################
  # Wait for System and Config Statuses Callbacks
  def systemStatusCb(self,msg):
        self.active_nodes = msg.active_nodes
        self.active_topics = msg.active_topics
        self.active_topic_types = msg.active_topic_types
        self.active_services = msg.active_services

  def initCb(self,do_updates = False):
    if self.node_if is not None:

        #####################
        ###Servo
        #####################
        
        self.selected_servo = self.node_if.get_param('selected_servo')
        self.speed_ratio = self.node_if.get_param('speed_ratio')
        self.pan_speed_ratio = self.node_if.get_param('pan_speed_ratio')
        self.tilt_speed_ratio = self.node_if.get_param('tilt_speed_ratio')



        #####################
        ###Scan
        #####################

        # scan_pan_enabled = self.node_if.get_param('scan_pan_enabled')
        # scan_tilt_enabled = self.node_if.get_param('scan_tilt_enabled')
        self.scan_pan_min_deg = self.node_if.get_param('scan_pan_min_deg')
        self.scan_pan_max_deg = self.node_if.get_param('scan_pan_max_deg')
        self.scan_tilt_min_deg = self.node_if.get_param('scan_tilt_min_deg')
        self.scan_tilt_max_deg = self.node_if.get_param('scan_tilt_max_deg')
        # self.setScanPan(scan_pan_enabled)
        # self.setScanTilt(scan_tilt_enabled)


        #####################
        ###Track
        #####################


        # track_pan_enabled = self.node_if.get_param('track_pan_enabled')
        # track_tilt_enabled = self.node_if.get_param('track_tilt_enabled')
        self.track_pan_min_deg = self.node_if.get_param('track_pan_min_deg')
        self.track_pan_max_deg = self.node_if.get_param('track_pan_max_deg')
        self.track_tilt_min_deg = self.node_if.get_param('track_tilt_min_deg')
        self.track_tilt_max_deg = self.node_if.get_param('track_tilt_max_deg')
        self.track_reset_time_sec = self.node_if.get_param('track_reset_time_sec')
        self.track_move_ratio = self.node_if.get_param('track_move_ratio')
        self.track_goal_deg = self.node_if.get_param('track_goal_deg')
        self.track_move_deg = self.node_if.get_param('track_move_deg')
        # self.setTrackPan(track_pan_enabled)
        # self.setTrackTilt(track_tilt_enabled)


        #####################
        ###Stab
        #####################

        self.selected_stab_source = self.node_if.get_param('selected_stab_source')

        stab_processes_dict =  self.node_if.get_param('stab_processes_dict')
        stab_processes_dict = nepi_stab_sv.update_processes_dict(stab_processes_dict)
        self.stab_processes_dict = stab_processes_dict

        selected_stab_process = self.node_if.get_param('selected_stab_process')
        if selected_stab_process in stab_processes_dict.keys():
            self.selected_stab_process = selected_stab_process
        else:
            self.selected_stab_process = list(stab_processes_dict.keys())[0]
        self.stab_process_ready = True

        self.stab_pan_speed_start = self.pan_speed_ratio
        self.stab_tilt_speed_start = self.tilt_speed_ratio
        
        stab_pan_enabled = self.node_if.get_param('stab_pan_enabled')
        stab_tilt_enabled = self.node_if.get_param('stab_tilt_enabled')
        self.setStabPan(stab_pan_enabled)
        self.setStabTilt(stab_tilt_enabled)

      


        #####################
        ###Tracking
        #####################
        self.tracking_manages_targeting = self.node_if.get_param('tracking_manages_targeting')
        tracking_dict = self.node_if.get_param('tracking_dict')
        blank_dict = copy.deepcopy(nepi_track.BLANK_SETTINGS_DICT)
        if tracking_dict is not None:
            for key in blank_dict.keys():
                if key not in tracking_dict.keys():
                    tracking_dict[key] = blank_dict[key]
        else:
           tracking_dict = blank_dict
        self.tracking_dict = tracking_dict
        self.tracking_targets_topic = tracking_dict['targets_topic']

        #####################
        ###Predict
        #####################
        # self.predict_source_topic = self.node_if.get_param('predict_source_topic')
        # predict_settings_dict = self.node_if.get_param('predict_settings_dict')
        
        # if predict_settings_dict is not None:
        #     blank_dict = copy.deepcopy(nepi_predict.PREDICT_DICT)
        #     for key in blank_dict.keys():
        #         if key not in predict_settings_dict.keys():
        #             predict_settings_dict[key] = blank_dict[key]
        #     process_list = list(predict_settings_dict['process_dict'].keys())
        #     blank_dict = copy.deepcopy(nepi_predict.BLANK_PROCESS_DICT)
        #     for process_name in process_list:
        #         for key in blank_dict.keys():
        #             if key not in predict_settings_dict['process_dict'][process_name].keys():
        #                 predict_settings_dict['process_dict'][process_name][key] = blank_dict[key]
        # else:
        #    predict_settings_dict = copy.deepcopy(nepi_predict.BLANK_PREDICT_DICT)
        # self.predict_settings_dict = predict_settings_dict

        #####################
        ###Imaging
        #####################
        self.num_windows = self.node_if.get_param('num_windows')
        self.selected_image_topics = self.node_if.get_param('selected_image_topics')
        self.single_image_topic = self.node_if.get_param('single_image_topic')
        if self.single_image_topic == 'None' and self.selected_image_topics[0] != 'None':
            self.single_image_topic = self.selected_image_topics[0]
            self.node_if.set_param('single_image_topic',self.single_image_topic)


    if do_updates == True:
      pass
    self.publish_status()

  def resetCb(self,do_updates = True):
      self.msg_if.pub_warn("Reseting")
      if self.node_if is not None:
        pass
      if do_updates == True:
        pass
      self.initCb(do_updates = do_updates)


  def factoryResetCb(self,do_updates = True):
      self.msg_if.pub_warn("Factory Reseting")
      if self.node_if is not None:
        pass
      if do_updates == True:
        pass
      self.initCb(do_updates = do_updates)


  def imageUpdateCb(self):
    self.img_needs_update = True


  def updaterCb(self,timer):
    needs_publish = False
    ##############

    selected_servo = copy.deepcopy(self.selected_servo)
    last_available = copy.deepcopy(self.available_servos)

    topics = nepi_sdk.find_topics_by_msg('DeviceSVXStatus', topics_list = self.active_topics, types_list = self.active_topic_types)
    available_servos = []
    for topic in topics:
      available_servos.append(topic.replace('/status',''))
    if available_servos != last_available:
      self.available_servos = available_servos
      needs_publish = True

    ####################
    if self.sv_connected_topic is not None:
      if self.sv_connected_topic not in self.available_servos:
        success = self.unsubscribe_sv_topic()
    if selected_servo == 'None' and len(self.available_servos) > 0:
        self.selected_servo = self.available_servos[0]
    needs_publish = True

    was_connected = copy.deepcopy(self.sv_connected)
    if self.selected_servo in self.available_servos and self.sv_connected_topic != selected_servo:
        success = self.subscribe_sv_topic(self.selected_servo)
    elif self.sv_connect_if is not None:
        self.sv_connected = self.sv_connect_if.check_connection()
        if self.sv_connected == True:
            limits = self.sv_connect_if.get_servo_soft_limits()
            #self.msg_if.pub_warn("setting scan limits: " + str(limits))
            if limits is not None:
                self.min_pan_softstop_deg = round(limits[0], 0) + self.LIMIT_PADDING
                self.max_pan_softstop_deg = round(limits[1], 0) - self.LIMIT_PADDING
                self.min_tilt_softstop_deg = round(limits[2], 0) + self.LIMIT_PADDING
                self.max_tilt_softstop_deg = round(limits[3], 0) - self.LIMIT_PADDING
                self.setScanPanWindow(self.scan_pan_min_deg, self.scan_pan_max_deg)
                self.setScanTiltWindow(self.scan_tilt_min_deg, self.scan_tilt_max_deg)
        needs_publish = True
    else:
        self.sv_connected = False

    ######################
    topics = nepi_sdk.find_topics_by_msg('Image', topics_list = self.active_topics, types_list = self.active_topic_types)
    available_image_topics = []
    image_priority_list = []
    for topic in topics:
      available_image_topics.append(topic)   
      image_name = os.path.basename(topic)
      for priority_option in self.image_priority_dict.keys(): 
         priority_name = self.image_priority_dict[priority_option]
         if priority_name == image_name and priority_option not in image_priority_list:
            image_priority_list.append(priority_option)
    self.available_image_topics = available_image_topics
    self.image_priority_list = image_priority_list
      
    ##################
    # Get settings from param server
    # if needs_publish == True:
    #   self.publish_status()
    nepi_sdk.start_timer_process(1.0, self.updaterCb, oneshot = True)


  def updaterServoCb(self,timer):
    #self.msg_if.pub_warn("Updater Called")
    if self.sv_connect_if is not None:
      self.current_position = self.sv_connect_if.get_servo_position()
      #self.msg_if.pub_warn("current_position: " + str(current_position))



  def get_image_priority_topic(self,image_topic,image_priority):
    priority_topic = image_topic
    if image_priority in self.image_priority_dict.keys():
      base_topic = os.path.dirname(image_topic)
      priority_name = self.image_priority_dict[image_priority]
      check_topic = os.path.join(base_topic,priority_name)
      topics = nepi_sdk.find_topics_by_msg('Image', topics_list = self.active_topics, types_list = self.active_topic_types)
      if check_topic in  topics:
         priority_topic = check_topic
    return priority_topic
     


  ##############################
  ## Node PT Commands

  def servoHomeCb(self, msg):
      if self.sv_connect_if is not None:
          self.sv_connect_if.go_home()

  def panHomeCb(self, msg):
      self.stopPanControls()
      if self.pan_stabing == True:
          self.stab_position[0] = 0
      elif self.sv_connect_if is not None:
          self.sv_connect_if.goto_to_pan_position(0.0)

  def tiltHomeCb(self, msg):
      self.stopTiltControls()
      if self.tilt_stabing == True:
          self.stab_position[1] = 0
      elif self.sv_connect_if is not None:
          self.sv_connect_if.goto_to_tilt_position(0.0)

  def stopPanControls(self):
    self.scan_pan_enabled = False
    self.pan_track_hold = False
    self.track_pan_enabled = False
    self.click_pan_enabled = True
    #self.stab_pan_enabled = False
    if self.sv_connect_if is not None:
        #self.sv_connect_if.set_pan_speed_ratio(1)
        sv_status_msg = self.sv_connect_if.get_status_msg()
        self.goto_position[0] = sv_status_msg.pan_now_deg



  def stopTiltControls(self):
    self.scan_tilt_enabled = False
    self.scan_tilt_track_hold = False
    self.track_tilt_enabled = False
    self.click_tilt_enabled = True
    #self.stab_tilt_enabled = False
    if self.sv_connect_if is not None:
        #self.sv_connect_if.set_tilt_speed_ratio(1)
        sv_status_msg = self.sv_connect_if.get_status_msg()
        self.goto_position[1] = sv_status_msg.tilt_now_deg


  def getPanClickEnabled(self):
     return (self.click_pan_enabled == True) and (self.scan_pan_enabled == False and self.track_pan_enabled == False and self.stab_pan_enabled == False)

  def getTiltClickEnabled(self):
     return (self.click_tilt_enabled == True) and (self.scan_tilt_enabled == False and self.track_tilt_enabled == False and self.stab_tilt_enabled == False)


  ##########################################
  # SCAN

  def setScanPanCb(self, msg):
        enabled = msg.data
        self.msg_if.pub_info("Setting scan pan: " + str(enabled))
        self.setScanPan(enabled)


  def setScanPan(self,enabled):
        was_scanning = copy.deepcopy(self.scan_pan_enabled)
        self.stab_pan_enabled = False
        self.scan_pan_enabled = enabled
        if enabled == True:
            self.goto_position[0] = 0.0
        if (was_scanning == True and enabled == False and self.track_pan_enabled == False and self.sv_connect_if is not None):
            self.sv_connect_if.set_pan_speed_ratio(self.pan_speed_ratio)
            self.sv_connect_if.goto_to_pan_position(0.0)
        self.publish_status()
        self.node_if.set_param('scan_pan_enabled', enabled)
        


  def setScanTiltCb(self, msg):
        enabled = msg.data
        self.msg_if.pub_info("Setting scan tilt: " + str(enabled))
        self.setScanTilt(enabled)



  def setScanTilt(self,enabled):
        was_scanning = copy.deepcopy(self.scan_tilt_enabled)
        self.stab_tilt_enabled = False
        self.scan_tilt_enabled = enabled
        if enabled == True:
            self.goto_position[1] = 0.0
        if (was_scanning == True and enabled == False and self.track_tilt_enabled == False and self.sv_connect_if is not None):
               self.sv_connect_if.set_tilt_speed_ratio(self.tilt_speed_ratio)
               self.sv_connect_if.goto_to_tilt_position(0.0)      
        self.publish_status()  
        self.node_if.set_param('scan_tilt_enabled', self.scan_tilt_enabled)


  def setScanPanSpeedRatioCb(self, msg):
      self.setScanPanSpeedRatio(msg.data)

  def setScanPanSpeedRatio(self, speed_ratio):
        speed_ratio = nepi_utils.check_ratio(speed_ratio)
        if speed_ratio < 0.1:
            speed_ratio = 0.1
        self.scan_pan_speed_ratio = speed_ratio
        if self.scan_pan_enabled == True and self.track_pan_enabled == False and self.sv_connect_if is not None:
            self.sv_connect_if.set_pan_speed_ratio(speed_ratio)
            self.sv_connect_if.goto_to_pan_position(self.goto_position[0])    
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('scan_pan_speed_ratio', speed_ratio)

  def setScanTiltSpeedRatioCb(self, msg):
      self.setScanTiltSpeedRatio(msg.data)

  def setScanTiltSpeedRatio(self, speed_ratio):
        speed_ratio = nepi_utils.check_ratio(speed_ratio)
        if speed_ratio < 0.1:
            speed_ratio = 0.1
        self.scan_tilt_speed_ratio = speed_ratio
        if self.scan_tilt_enabled == True and self.track_tilt_enabled == False and self.sv_connect_if is not None:
            self.sv_connect_if.set_pan_speed_ratio(speed_ratio)
            self.sv_connect_if.goto_to_tilt_position(self.goto_position[1])    
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('scan_tilt_speed_ratio', speed_ratio)



  def setScanPanWindowCb(self, msg):
      adj_min_deg = msg.start_range
      adj_max_deg = msg.stop_range
      if adj_min_deg > adj_max_deg:
        self.msg_if.pub_info("invalid range: " + "%.2f" % adj_min_deg  + " " + "%.2f" % adj_max_deg )
      else:
        self.msg_if.pub_info("Setting scan pan limits to: " + "%.2f" % adj_min_deg  + " " + "%.2f" % adj_max_deg )
        self.setScanPanWindow(adj_min_deg,adj_max_deg)

  def setScanPanWindow(self, min_deg, max_deg):
        if max_deg > min_deg and abs(max_deg - min_deg) >= self.MIN_SCAN_ANGLE:
            if max_deg > self.max_pan_softstop_deg:
                max_deg = self.max_pan_softstop_deg
            if min_deg < self.min_pan_softstop_deg:
                min_deg = self.min_pan_softstop_deg
            self.scan_pan_min_deg = min_deg
            self.scan_pan_max_deg = max_deg
            #self.msg_if.pub_info("Scan Pan limits set to: " + "%.2f" % min_deg  + " " + "%.2f" % max_deg )
            self.publish_status()
            self.node_if.set_param('scan_pan_min_deg', min_deg)
            self.node_if.set_param('scan_pan_max_deg', max_deg)
            


  def setScanTiltWindowCb(self, msg):
      adj_min_deg = msg.start_range
      adj_max_deg = msg.stop_range
      self.msg_if.pub_info("Setting scan tilt limits to: " + "%.2f" % adj_min_deg  + " " + "%.2f" % adj_max_deg )
      self.setScanTiltWindow(adj_min_deg,adj_max_deg)


  def setScanTiltWindow(self, min_deg, max_deg):
      if max_deg > min_deg and abs(max_deg - min_deg) >= self.MIN_SCAN_ANGLE:
          if max_deg > self.max_tilt_softstop_deg:
              max_deg = self.max_tilt_softstop_deg
          if min_deg < self.min_tilt_softstop_deg:
              min_deg = self.min_tilt_softstop_deg
          self.scan_tilt_min_deg = min_deg
          self.scan_tilt_max_deg = max_deg
          #self.msg_if.pub_info("Scan Tilt limits set to: " + "%.2f" % min_deg  + " " + "%.2f" % max_deg )
          self.publish_status()
          self.node_if.set_param('scan_tilt_min_deg', min_deg)
          self.node_if.set_param('scan_tilt_max_deg', max_deg)



  ##########################################
  # TRACK

  def setTrackPanCb(self, msg):
        enabled = msg.data
        self.msg_if.pub_info("Setting track pan: " + str(enabled))
        self.setTrackPan(enabled)


  def setTrackPan(self,enabled):
        if enabled == True:
            #self.scan_pan_enabled = False
            self.stab_pan_enabled = False           
        self.track_pan_enabled = enabled
        self.pan_track_hold = enabled
        self.stab_pan_enabled = False
        self.setTrackingEnable(enabled or self.track_tilt_enabled)
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('track_pan_enabled', self.track_pan_enabled)

  def setTrackTiltCb(self, msg):
        enabled = msg.data
        self.msg_if.pub_info("Setting track tilt: " + str(enabled))
        self.setTrackTilt(enabled)

  def setTrackTilt(self,enabled):
        if enabled == True:
            #self.scan_tilt_enabled = False
            pass
        self.track_tilt_enabled = enabled
        self.tilt_track_hold = enabled
        self.stab_tilt_enabled = False
        self.setTrackingEnable(enabled or self.track_pan_enabled)
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('track_tilt_enabled', self.track_tilt_enabled)


  def setTrackPanWindowCb(self, msg):
      adj_min_deg = msg.start_range
      adj_max_deg = msg.stop_range
      if adj_min_deg > adj_max_deg:
        self.msg_if.pub_info("invalid range: " + "%.2f" % adj_min_deg  + " " + "%.2f" % adj_max_deg )
      else:
        self.msg_if.pub_info("Setting track pan limits to: " + "%.2f" % adj_min_deg  + " " + "%.2f" % adj_max_deg )
        self.setTrackPanWindow(adj_min_deg,adj_max_deg)

  def setTrackPanWindow(self, min_deg, max_deg):
        if max_deg > min_deg:
            if max_deg > self.max_pan_softstop_deg:
                max_deg = self.max_pan_softstop_deg
            if min_deg < self.min_pan_softstop_deg:
                min_deg = self.min_pan_softstop_deg
            self.track_pan_min_deg = min_deg
            self.track_pan_max_deg = max_deg
            self.msg_if.pub_info("Track Pan limits set to: " + "%.2f" % min_deg  + " " + "%.2f" % max_deg )
            self.publish_status()
            self.node_if.set_param('track_pan_min_deg', min_deg)
            self.node_if.set_param('track_pan_max_deg', max_deg)
            


  def setTrackTiltWindowCb(self, msg):
      adj_min_deg = msg.start_range
      adj_max_deg = msg.stop_range
      self.msg_if.pub_info("Setting track tilt limits to: " + "%.2f" % adj_min_deg  + " " + "%.2f" % adj_max_deg )
      self.setTrackTiltWindow(adj_min_deg,adj_max_deg)


  def setTrackTiltWindow(self, min_deg, max_deg):
      if max_deg > min_deg:
          if max_deg > self.max_tilt_softstop_deg:
              max_deg = self.max_tilt_softstop_deg
          if min_deg < self.min_tilt_softstop_deg:
              min_deg = self.min_tilt_softstop_deg
          self.track_tilt_min_deg = min_deg
          self.track_tilt_max_deg = max_deg
          self.msg_if.pub_info("Track Tilt limits set to: " + "%.2f" % min_deg  + " " + "%.2f" % max_deg )
          self.publish_status()
          self.node_if.set_param('track_tilt_min_deg', min_deg)
          self.node_if.set_param('track_tilt_max_deg', max_deg)

  def setTrackResetTimeSecCb(self, msg):
      reset_time = msg.data
      self.setTrackResetTimeSec(reset_time)

  def setTrackResetTimeSec(self,reset_time):
        if reset_time < 1:
            reset_time = 1
        if reset_time > self.TRACK_MAX_RESET_SEC:
            reset_time = self.TRACK_MAX_RESET_SEC
        self.msg_if.pub_info("Setting track reset time to: " + str(reset_time))
        self.track_reset_time_sec = reset_time
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('track_reset_time_sec', reset_time)
            #self.node_if.save_config()

  def setTrackMoveRatioCb(self, msg):
      ratio = msg.data
      self.setTrackMoveRatio(ratio)


  def setTrackMoveRatio(self,ratio):
        ratio = nepi_utils.check_ratio(ratio)
        self.msg_if.pub_info("Setting track move ratio to: " + str(ratio))
        self.track_move_ratio = ratio
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('track_move_ratio', ratio)
            ##self.node_if.save_config()

  def setTrackGoalDegCb(self, msg):
      goal_deg = msg.data
      self.setTrackGoalDeg(goal_deg)

  def setTrackGoalDeg(self, goal_deg):
        if goal_deg < 0:
            goal_deg = 0
        self.msg_if.pub_info("Setting track goal deg to: " + str(goal_deg))
        self.track_goal_deg = goal_deg
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('track_goal_deg', goal_deg)
            #self.node_if.save_config()

  def setTrackMoveDegCb(self, msg):
      move_deg = msg.data
      self.setTrackMoveDeg(move_deg)

  def setTrackMoveDeg(self, move_deg):
        if move_deg < 0:
            move_deg = 0
        self.msg_if.pub_info("Setting track move deg to: " + str(move_deg))
        self.track_move_deg = move_deg
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('track_move_deg', move_deg)
            #self.node_if.save_config()



  def setSpeedRatioCb(self, msg):
      self.setSpeedRatio(msg.data)

  def setSpeedRatio(self, speed_ratio):
        speed_ratio = nepi_utils.check_ratio(speed_ratio)
        if speed_ratio < 0.1:
            speed_ratio = 0.1
        self.speed_ratio = speed_ratio
        self.pan_speed_ratio = speed_ratio
        self.tilt_speed_ratio = speed_ratio
        if self.sv_connect_if is not None:
            self.sv_connect_if.set_speed_ratio(speed_ratio)
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('speed_ratio', self.speed_ratio)
            self.node_if.set_param('pan_speed_ratio', self.pan_speed_ratio)
            self.node_if.set_param('tilt_speed_ratio', self.tilt_speed_ratio)

  def setPanSpeedRatioCb(self, msg):
      self.setPanSpeedRatio(msg.data)

  def setPanSpeedRatio(self, speed_ratio):
        speed_ratio = nepi_utils.check_ratio(speed_ratio)
        if speed_ratio < 0.1:
            speed_ratio = 0.1
        self.pan_speed_ratio = speed_ratio
        if self.sv_connect_if is not None:
            self.sv_connect_if.set_pan_speed_ratio(speed_ratio)
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('pan_speed_ratio', self.pan_speed_ratio)

  def setTiltSpeedRatioCb(self, msg):
      self.setTiltSpeedRatio(msg.data)

  def setTiltSpeedRatio(self, speed_ratio):
        speed_ratio = nepi_utils.check_ratio(speed_ratio)
        if speed_ratio < 0.1:
            speed_ratio = 0.1
        self.tilt_speed_ratio = speed_ratio
        if self.sv_connect_if is not None:
            self.sv_connect_if.set_tilt_speed_ratio(speed_ratio)
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('tilt_speed_ratio', self.tilt_speed_ratio)

  '''
  def setSinPanCb(self, msg):
      enabled = msg.data
      self.scan_pan_last_time = nepi_utils.get_time()
      self.msg_if.pub_info("Setting Sin pan: " + str(enabled))
      self.setSinPan(enabled)


  def setSinPan(self,enabled):
      self.sin_pan_enabled = enabled
      self.publish_status()
      if enabled == False and self.sin_tilt_enabled == False and self.sin_pan_enabled == False and self.setSpeedRatioCb is not None:
          self.msg_if.pub_info("2")
          self.setSpeedRatioCb(self.speed_ratio)
      self.node_if.set_param('sin_pan_enabled', self.sin_pan_enabled)
      
  '''
    ##############################
    # Tracking
    #############################


  def setTrackManagesTargetingCb(self, msg):
      value = msg.data
      self.setTrackManagesTargeting(value)

  def setTrackManagesTargeting(self,value):
        self.msg_if.pub_info("Setting track move ratio to: " + str(value))
        self.tracking_manages_targeting = value
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('tracking_manages_targeting', self.tracking_manages_targeting)
            #self.node_if.save_config()

  def setTrackTargetsTopicCb(self, msg):
      value = msg.data
      self.setTrackTargetsTopic(value)

  def setTrackTargetsTopic(self,value):
        self.msg_if.pub_info("Setting track targets topic to: " + str(value))
        self.tracking_dict['targets_topic'] = value
        self.tracking_targets_topic = value
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('tracking_dict', self.tracking_dict)
            #self.node_if.save_config()


  def setTrackSourceTopicCb(self, msg):
      value = msg.data
      self.setTrackSourceTopic(value)

  def setTrackSourceTopic(self,value):
        self.msg_if.pub_info("Setting track source ratio to: " + str(value))
        self.tracking_dict['source_topic'] = value
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('tracking_dict', self.tracking_dict)
            #self.node_if.save_config()


  def setTrackClassFilterCb(self, msg):
      value = msg.data
      self.setTrackClassFilter(value)

  def setTrackClassFilter(self,value):
        pass
        # self.msg_if.pub_info("Setting track class filter to: " + str(value))
        # self.tracking_dict['class_filter'] = value
        # self.publish_status()
        # if self.node_if is not None:
        #     self.node_if.set_param('tracking_dict', self.tracking_dict)
        #     #self.node_if.save_config()


  def setTrackThresholdFilterCb(self, msg):
      ratio = msg.data
      self.setTrackThresholdFilter(ratio)

  def setTrackThresholdFilter(self,ratio):
        ratio = nepi_utils.check_ratio(ratio)
        #self.msg_if.pub_info("Setting track threshold ratio to: " + str(ratio))
        last_val = copy.deepcopy(self.tracking_dict['threshold_filter'])
        self.tracking_dict['threshold_filter'] = ratio
        if last_val != ratio:
            self.publish_status()
            if self.node_if is not None:
                self.node_if.set_param('tracking_dict', self.tracking_dict)
                ##self.node_if.save_config()


  def setTrackBestFilterCb(self, msg):
      value = msg.data
      self.setTrackBestFilter(value)

  def setTrackBestFilter(self,value):
        #self.msg_if.pub_info("Setting track move ratio to: " + str(ratio))
        self.tracking_dict['best_filter'] = value
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('tracking_dict', self.tracking_dict)
            ##self.node_if.save_config()


    ##############################
    # Proccesses
    #############################


  def scanPanProcess(self,timer):
      #self.msg_if.pub_warn("Starting Pan Scan Process") 
      #self.msg_if.pub_warn("current_position: " + str(self.current_position)) 
      cur_time = nepi_utils.get_time()
      scan_time = None

      if self.current_position == None or self.sv_connected == False:
        self.pan_scanning = False
      else:
        if self.scan_pan_enabled == False or self.pan_track_hold == True:
            self.pan_scanning = False
        elif self.sv_connect_if is not None:
            self.pan_scanning = True


            pan_cur = self.current_position[0]
            if self.goto_position[0] != self.scan_pan_min_deg and self.goto_position[0] != self.scan_pan_max_deg:
                #self.msg_if.pub_warn("goto pan scan pos: " + str(self.scan_pan_min_deg)) 
                self.goto_position[0] = self.scan_pan_min_deg
                self.sv_connect_if.set_pan_speed_ratio(self.scan_pan_speed_ratio)
                self.sv_connect_if.goto_to_pan_position(self.scan_pan_min_deg)  
              
            elif (pan_cur < (self.scan_pan_min_deg + self.SCAN_SWITCH_DEG)):
                last_time = self.scan_pan_last_time
                if last_time is not None:
                    scan_time =  cur_time - self.scan_pan_last_time
                self.scan_pan_last_time = nepi_utils.get_time()

                #self.msg_if.pub_warn("goto pan pos: " + str(self.scan_pan_max_deg)) 
                self.goto_position[0] = self.scan_pan_max_deg
                self.sv_connect_if.set_pan_speed_ratio(self.scan_pan_speed_ratio)
                self.sv_connect_if.goto_to_pan_position(self.scan_pan_max_deg)

                
            elif (pan_cur > (self.scan_pan_max_deg - self.SCAN_SWITCH_DEG)):
                last_time = self.scan_pan_last_time
                if last_time is not None:
                    scan_time =  cur_time - self.scan_pan_last_time
                self.scan_pan_last_time = nepi_utils.get_time()
                self.goto_position[0] = self.scan_pan_min_deg
                self.sv_connect_if.set_pan_speed_ratio(self.scan_pan_speed_ratio)
                self.sv_connect_if.goto_to_pan_position(self.scan_pan_min_deg)
            elif (self.sv_connect_if.check_pan_moving() == False):
                self.sv_connect_if.set_pan_speed_ratio(self.scan_pan_speed_ratio)
                self.sv_connect_if.goto_to_pan_position(self.goto_position[0])


            if scan_time is not None:
                self.scan_pan_times.pop(0)
                self.scan_pan_times.append(scan_time)
                
                # Calc scan pan times and sin
                scan_pan_times = copy.deepcopy(self.scan_pan_times)
                times = [x for x in scan_pan_times if x != 0]
                scan_pan_time = 0
                if len(times) > 0:
                    scan_pan_time = sum(times) / len(times)
                self.scan_pan_time = scan_time # scan_pan_time

                # sin_len = math.ceil(scan_pan_time) *2
                # self.scan_pan_sins = list( (np.sin(  (np.linspace(0,1,sin_len)*4*math.pi) - (math.pi)/2) + 1  ) /2 )
                # self.scan_pan_sin_ind = 0
                # #self.msg_if.pub_warn("updated pan sin " + str(self.scan_pan_sins))



  def scanTiltProcess(self,timer):
      #self.msg_if.pub_warn("Starting Tilt Scan Process") 

      cur_time = nepi_utils.get_time()
      scan_time = None
      if self.current_position == None or self.sv_connected == False:
        self.tilt_scanning = False
      else:
        if self.scan_tilt_enabled == False or self.tilt_track_hold == True:
            self.tilt_scanning = False
        elif self.sv_connect_if is not None:
            self.tilt_scanning = True


            tilt_cur = self.current_position[1]
            if self.goto_position[1] != self.scan_tilt_min_deg and self.goto_position[1] != self.scan_tilt_max_deg:
                self.goto_position[1] = self.scan_tilt_min_deg
                self.sv_connect_if.set_tilt_speed_ratio(self.scan_tilt_speed_ratio)
                self.sv_connect_if.goto_to_tilt_position(self.scan_tilt_min_deg)  
            elif (tilt_cur < (self.scan_tilt_min_deg + self.SCAN_SWITCH_DEG)):
                last_time = self.scan_tilt_last_time
                if last_time is not None:
                    scan_time =  cur_time - self.scan_tilt_last_time
                self.scan_tilt_last_time = nepi_utils.get_time()
                self.goto_position[1] = self.scan_tilt_max_deg
                self.sv_connect_if.set_tilt_speed_ratio(self.scan_tilt_speed_ratio)
                self.sv_connect_if.goto_to_tilt_position(self.scan_tilt_max_deg)

            elif (tilt_cur > (self.scan_tilt_max_deg - self.SCAN_SWITCH_DEG)):
                last_time = self.scan_tilt_last_time
                if last_time is not None:
                    scan_time =  cur_time - self.scan_tilt_last_time
                self.scan_tilt_last_time = nepi_utils.get_time()
                self.goto_position[1] = self.scan_tilt_min_deg
                self.sv_connect_if.set_tilt_speed_ratio(self.scan_tilt_speed_ratio)
                self.sv_connect_if.goto_to_tilt_position(self.scan_tilt_min_deg)
            elif (self.sv_connect_if.check_tilt_moving() == False):
                self.sv_connect_if.set_tilt_speed_ratio(self.scan_tilt_speed_ratio)
                self.sv_connect_if.goto_to_tilt_position(self.goto_position[1])

            if scan_time is not None:
                self.scan_tilt_times.pop(0)
                self.scan_tilt_times.append(scan_time)
                
                # Calc scan tilt times and sin
                scan_tilt_times = copy.deepcopy(self.scan_tilt_times)
                times = [x for x in scan_tilt_times if x != 0]
                scan_tilt_time = 0
                if len(times) > 0:
                    scan_tilt_time = sum(times) / len(times)
                self.scan_tilt_time = scan_time # scan_tilt_time

                # sin_len = math.ceil(scan_tilt_time) *2
                # self.scan_tilt_sins = list( (np.sin(  (np.linspace(0,1,sin_len)*4*math.pi) - (math.pi)/2) + 1  ) /2 )
                # self.scan_tilt_sin_ind = 0
                # self.msg_if.pub_warn("updated tilt sin " + str(self.scan_tilt_sins))



  def trackPanProcess(self,timer):
    if self.track_pan_enabled == False or self.tracking_targets_connected == False:
           self.pan_tracking = False
    else:
          if self.current_position == None or self.pan_stabing == True:
            self.pan_tracking = False
          else:
            pan_cur = self.current_position[0]
            track_dict = copy.deepcopy(self.track_pan_dict)
            last_error = copy.deepcopy(self.track_pan_error)
            self.track_pan_dict = None
            if track_dict is not None:
                #self.msg_if.pub_warn("Got track target dict " + str(track_dict))
                self.pan_track_hold = True
                self.pan_tracking = True
                self.last_track_pan_time = nepi_utils.get_time()
                pan_error = track_dict['azimuth_deg']
                #self.msg_if.pub_warn("Got target angle " + str(pan_error)) 
                self.track_pan_error = round(pan_error,1)
                #self.msg_if.pub_warn("Got track angle " + str(self.track_pan_error)) 
                if abs(self.track_pan_error) > self.track_goal_deg:
                  #self.msg_if.pub_warn("Tracking to pan error " + str(self.track_pan_error))    
                  pan_to_goal = round(pan_cur + self.track_pan_error  * self.track_move_ratio, 0)
                  #self.msg_if.pub_warn("Moving to pan position " + str(pan_to_goal))  
                  if abs(pan_to_goal - self.goto_position[0]) > self.track_move_deg:
                      self.goto_position[0] = pan_to_goal
                      self.sv_connect_if.goto_to_pan_position(pan_to_goal)
            else:
                track_last_time = nepi_utils.get_time() - self.track_last_time
                if track_last_time > self.track_reset_time_sec:
                    self.pan_tracking = False
                    self.track_pan_error = 0
                    goto = None
                    if self.scan_pan_enabled == True:
                        if self.pan_track_hold == True:
                            if last_error > 0:
                                goto = self.scan_pan_max_deg
                            else:
                               goto = self.scan_pan_min_deg
                    elif self.goto_position[0] != 0:
                        goto = 0
                    if goto is not None:
                        #self.msg_if.pub_warn("Resetting pan Scan Process to: " + str(self.goto_position[1]))
                        self.goto_position[0] = goto
                        self.sv_connect_if.goto_to_pan_position(goto)
                    self.tilt_track_hold = False
    nepi_sdk.start_timer_process(self.TRACK_UPDATE_INTERVAL, self.trackPanProcess, oneshot = True)
      
                
      

  def trackTiltProcess(self,timer):
    if self.track_tilt_enabled == False or self.tracking_targets_connected == False:
           self.tilt_tracking = False
    else:
          if self.current_position == None or self.tilt_stabing == True:
            self.tilt_tracking = False
          else:
            tilt_cur = self.current_position[1]
            track_dict = copy.deepcopy(self.track_tilt_dict)
            last_error = copy.deepcopy(self.track_tilt_error)
            self.track_tilt_dict = None
            if track_dict is not None:
                self.tilt_track_hold = True
                self.tilt_tracking = True
                tilt_error = track_dict['elevation_deg']
                self.track_tilt_error = round(tilt_error,1)
                self.last_track_tilt_time = nepi_utils.get_time()
                if abs(self.track_tilt_error) > self.track_goal_deg:
                  #self.msg_if.pub_warn("Got track tilt error " + str(tilt_error))    
                  tilt_to_goal = round(tilt_cur + self.track_tilt_error  * self.track_move_ratio, 0)
                  if abs(tilt_to_goal - self.goto_position[1]) > self.track_move_deg:
                      self.goto_position[1] = tilt_to_goal
                      self.sv_connect_if.goto_to_tilt_position(tilt_to_goal)
            else:          
                track_last_time = nepi_utils.get_time() - self.track_last_time
                if track_last_time > self.track_reset_time_sec:
                    self.tilt_tracking = False
                    self.track_tilt_error = 0
                    goto = None
                    if self.scan_tilt_enabled == True:
                        if self.tilt_track_hold == True:
                            if last_error > 0:
                                goto = self.scan_tilt_max_deg
                            else:
                               goto = self.scan_tilt_min_deg
                    elif self.goto_position[1] != 0:
                        goto = 0
                    if goto is not None:
                        #self.msg_if.pub_warn("Resetting Tilt Scan Process to: " + str(self.goto_position[1]))
                        self.goto_position[1] = goto
                        self.sv_connect_if.goto_to_tilt_position(goto)
                    self.tilt_track_hold = False
    nepi_sdk.start_timer_process(self.TRACK_UPDATE_INTERVAL, self.trackTiltProcess, oneshot = True)                    
                


  def filter_by_range_angles(self,targets_dict_list):
    ################
    # Filter by min max range and angles
    filtered_dict_list = []
    cur_position = copy.deepcopy(self.current_position)
    if cur_position is not None:
      [cur_pan,cur_tilt] = [cur_position[0],cur_position[1]]
      range_min = self.track_range_min_m
      range_max = self.track_range_max_m
      pan_min = self.scan_pan_min_deg #track_pan_min_deg
      pan_max = self.scan_pan_max_deg #track_pan_max_deg
      tilt_min = self.scan_tilt_min_deg #track_tilt_min_deg
      tilt_max = self.scan_tilt_max_deg #track_tilt_max_deg

      for target_dict in targets_dict_list:
          target_valid = True
          range_m = target_dict['range_m']
          if (range_m < range_min or range_m > range_max) and range_m != -999:
            target_valid = False
          target_pan_angle = target_dict['azimuth_deg']
          pan_angle =  cur_pan + target_pan_angle
          if (pan_angle < pan_min or pan_angle > pan_max) and target_pan_angle != -999:
            target_valid = False
          target_tilt_angle = cur_pan + target_dict['elevation_deg']
          tilt_angle =  cur_tilt + target_tilt_angle
          if (tilt_angle < tilt_min or tilt_angle > tilt_max) and target_tilt_angle != -999:
            target_valid = False
          if target_valid == True:
            filtered_dict_list.append(target_dict)
          #self.msg_if.pub_warn("Range Angle Filter returned: " + str(target_dict['target_name']) + " : " + str(target_valid) )
          #self.msg_if.pub_warn(str([range_m,cur_pan,cur_tilt]))
          #self.msg_if.pub_warn(str([range_m,target_pan_angle,target_tilt_angle]))
          #self.msg_if.pub_warn(str([range_m,pan_angle,tilt_angle]))
    return filtered_dict_list  



  def setPanClickCb(self, msg):
      enabled = msg.data
      self.msg_if.pub_info("Setting Click Pan Enabled: " + str(enabled))
      self.setPanClick(enabled)

  def setPanClick(self,enabled):
      if self.click_pan_enabled == False:
          self.click_position = [0,0]
      if self.click_pan_enabled == True:
          self.stopPanControls()
          self.track_pan_error = 0

      self.click_pan_enabled = enabled
      self.publish_status()


  def setTiltClickCb(self, msg):
      enabled = msg.data
      self.msg_if.pub_info("Setting Click Tilt: " + str(enabled))
      #self.setTiltClick(enabled)

  def setTiltClick(self,enabled):
      if self.click_tilt_enabled == False:
          self.click_position = [0,0]
      if self.click_tilt_enabled == True:
          self.stopTiltControls()
          self.track_tilt_error = 0
      self.click_tilt_enabled = enabled
      self.publish_status()

  

  def selectTopicCb(self,msg):
    selected_servo = msg.data
    if selected_servo in self.available_servos:
      self.selected_servo = selected_servo
      self.publish_status()
      if self.node_if is not None:
        self.msg_if.pub_warn("selected_servo: " + str(selected_servo))
        self.node_if.set_param('selected_servo', selected_servo)
    

  def getPositionWithinSoftLimits(self, pan_deg, tilt_deg):
        pan_max = self.min_pan_softstop_deg
        pan_min = self.max_pan_softstop_deg
        tilt_min = self.min_tilt_softstop_deg
        tilt_max = self.max_tilt_softstop_deg
        if (pan_deg > pan_max):
            pan_deg = pan_max
        if (pan_deg < pan_min):
            pan_deg = pan_min
        if (tilt_deg > tilt_max):
            tilt_deg = tilt_max
        if (tilt_deg < tilt_min):
            tilt_deg = tilt_min
        return pan_deg,tilt_deg


  def subscribe_sv_topic(self, topic):
    self.msg_if.pub_warn("subscribe_sv_topic Called")

    success = False
    if self.sv_connect_if is not None:
      success = self.unsubscribe_sv_topic()

    sv_connect_if = ConnectSVXDeviceIF(namespace = topic,
                                       servoCb = self.servoCb,
                                       stopPanCb = self.stopPanCb,
                                       stopTiltCb = self.stopTiltCb,
                                       msg_if = self.msg_if
                                        )
    ready = sv_connect_if.wait_for_ready()
    if ready == True:
      self.sv_connect_if = sv_connect_if
      self.sv_connected_topic = topic
      self.msg_if.pub_warn("sv_connected_topic: " + str(self.sv_connected_topic))
      self.sv_connect_if.set_speed_ratio(self.speed_ratio)
      self.sv_connect_if.set_pan_speed_ratio(self.pan_speed_ratio)
      self.sv_connect_if.set_tilt_speed_ratio(self.tilt_speed_ratio)
    return success
  


  
  def unsubscribe_sv_topic(self):
    self.msg_if.pub_warn("unsubscribe_sv_topic Called")

    success = True
    if self.sv_connect_if is not None:
      success = self.sv_connect_if.unregister()
      self.sv_connected = False
      self.sv_connected_topic = None
      self.current_position = None
      nepi_sdk.sleep(1)
      self.sv_connect_if = None
    return success

  def servoCb(self, pan_deg, tilt_deg):
     self.current_position = [pan_deg, tilt_deg]
     #self.msg_if.pub_warn("PT position: " + str(self.current_position))

  def stopPanCb(self):
     #self.msg_if.pub_warn("Got Stop Pan Cb")
     self.stopPanControls()



  def stopTiltCb(self):
     #self.msg_if.pub_warn("Got Stop Tilt Cb")
     self.stopTiltControls()  







##########################
###Image Veiwer
##########################

  def mouseClickCb(self,msg):
      if msg.click_event == True:
        click_count = msg.click_count

        if click_count > 1:
            if self.num_windows == 1:
                self.setNumWindows(self.last_num_windows)
            else:
                image_index = msg.image_index
                image_topic = msg.image_topic
                if image_topic != 'None':
                    self.single_image_topic = image_topic
                    self.publish_status()
                    self.setNumWindows(1)
            
        else:
            self.click_position = [0,0]
            click_pan_enabled = self.getPanClickEnabled()
            click_tilt_enabled = self.getTiltClickEnabled()
            image_index = msg.image_index
            
            pixel = [msg.click.x, msg.click.y ]
            status_msg = msg.image_status_msg
            image_width = status_msg.width_px
            image_height = status_msg.height_px
            image_fov_horz = status_msg.width_deg
            image_fov_vert = status_msg.height_deg
            image_zoom_ratio = status_msg.zoom_ratio
            if image_width > 10 and image_height > 10 and image_fov_horz > 10 and image_fov_vert > 10 and image_zoom_ratio < 0.1:
                object_loc_x_ratio_from_center = float(pixel[0] - image_width/2) / float(image_width/2)
                object_loc_y_ratio_from_center = float(pixel[1] - image_height/2) / float(image_height/2)
                vert_angle_deg = (object_loc_y_ratio_from_center * float(image_fov_vert/2))
                horz_angle_deg = - (object_loc_x_ratio_from_center * float(image_fov_horz/2))
                self.click_position = [horz_angle_deg,vert_angle_deg]

            if click_pan_enabled == True:
                if self.current_position == None:
                    pass
                else:
                    pan_cur = self.current_position[0]
                    pan_to_goal = self.click_position[0] + pan_cur
                    self.msg_if.pub_warn("Pixel Selected, Going to Pan Pos " + str(pan_to_goal))#)
                    self.goto_position[0] = pan_to_goal
                    self.sv_connect_if.goto_to_pan_position(pan_to_goal)
            else: 
                self.msg_if.pub_warn("Pan Click Enabled is False")#)



            if click_tilt_enabled == True:
                if self.current_position == None:
                    pass
                else:
                    tilt_cur = self.current_position[1]
                    tilt_to_goal = self.click_position[1] + tilt_cur
                    self.msg_if.pub_warn("Pixel Selected, Going to Tilt Pos " + str(tilt_to_goal))#)
                    self.goto_position[1] = tilt_to_goal
                    self.sv_connect_if.goto_to_tilt_position(tilt_to_goal)
            else: 
                self.msg_if.pub_warn("Tilt Click Enabled is False")#)


  def setImageTopic1Cb(self,msg):
    self.msg_if.pub_info(str(msg))
    img_index = 0
    img_topic = msg.data
    if self.num_windows == 1 or self.single_image_topic == 'None':
        self.single_image_topic = msg.data
    if img_index < len(self.selected_image_topics):
      if self.num_windows > 1 or self.selected_image_topics[img_index] == 'None':
        self.selected_image_topics[img_index] = img_topic
    self.publish_status()
    if self.node_if is not None:
        self.node_if.set_param('selected_image_topics', self.selected_image_topics)
        #self.node_if.save_config()

  def setImageTopic2Cb(self,msg):
    self.msg_if.pub_info(str(msg))
    img_index = 1
    img_topic = msg.data
    if img_index < len(self.selected_image_topics):
      self.selected_image_topics[img_index] = img_topic
      self.publish_status()
      if self.node_if is not None:
        self.node_if.set_param('selected_image_topics', self.selected_image_topics)
        #self.node_if.save_config()

  def setImageTopic3Cb(self,msg):
    self.msg_if.pub_info(str(msg))
    img_index = 2
    img_topic = msg.data
    if img_index < len(self.selected_image_topics):
      self.selected_image_topics[img_index] = img_topic
      self.publish_status()
      if self.node_if is not None:
        self.node_if.set_param('selected_image_topics', self.selected_image_topics)
        #self.node_if.save_config()

  def setImageTopic4Cb(self,msg):
    self.msg_if.pub_info(str(msg))
    img_index = 3
    img_topic = msg.data
    if img_index < len(self.selected_image_topics):
      self.selected_image_topics[img_index] = img_topic
      self.publish_status()
      if self.node_if is not None:
        self.node_if.set_param('selected_image_topics', self.selected_image_topics)
        #self.node_if.save_config()
      
  def setNumWindowsCb(self,msg):
    self.msg_if.pub_info(str(msg))
    num_windows = msg.data
    self.setNumWindows(num_windows)

  def setNumWindows(self,num_windows):
    if num_windows > 0 and num_windows < 5:
      if num_windows == 1 and self.num_windows != 1:
         self.last_num_windows = copy.deepcopy(self.num_windows)
      self.num_windows = num_windows
      self.publish_status()
      if self.node_if is not None:
        self.node_if.set_param('num_windows', self.num_windows)
        #self.node_if.save_config()

  def setImagePriorityCb(self,msg):
    image_priority = msg.data
    self.single_image_topic = self.get_image_priority_topic(self.single_image_topic,image_priority)
    selected_image_topics = []
    for image_topic in self.selected_image_topics:
      selected_image_topics.append(self.get_image_priority_topic(image_topic,image_priority))
    self.selected_image_topics = selected_image_topics
    self.publish_status()
    if self.node_if is not None:
      self.node_if.set_param('single_image_topic', self.single_image_topic)
      self.node_if.set_param('selected_image_topics', self.selected_image_topics)
      #self.node_if.save_config()


##########################
### Tracking
##########################  

  def sendTargetsMsg(self,msg_name,msg):
        success = False
        if self.tracking_subpub_dict is not None:
            self.tracking_subpub_lock.acquire()
            try:
                nepi_sdk.publish_pub(self.tracking_subpub_dict[msg_name],msg)
                success = True
            except:
                pass
            self.tracking_subpub_lock.release()
        return success

  def sendTargetsConfig(self):
        msg = copy.deepcopy(self.targets_status_msg_start)
        if msg is None:
            self.msg_if.pub_warn("No original config to send")
        else:
            cur_tracking_source = self.tracking_dict['source_topic']
            if cur_tracking_source != 'None':
                if cur_tracking_source in self.tracking_available_sources and  cur_tracking_source not in msg.selected_sources:
                    msg.selected_sources.append(cur_tracking_source)
            # cur_class_source = self.tracking_dict['class_filter']
            # if cur_class_source in self.tracking_available_classes and  cur_tracking_source not in msg.selected_classes:
            #     msg.selected_classes.append(cur_class_source)
            # msg.selected_classes = self.tracking_available_classes
            #self.msg_if.pub_warn("Resetting Targeting source with original config: " + str(msg))
            self.sendTargetsMsg('config_pub',msg)
            

  def setTrackingEnable(self,enabled):
      if enabled == True:
        self.targets_status_msg_start = None
        nepi_sdk.sleep(1)
        self.tracking_enabled = True
        #self.sendTargetsMsg('save_pub',self.tracking_manages_targeting)
        self.sendTargetsMsg('save_pub',False)
        nepi_sdk.sleep(1)
        self.sendTargetsMsg('enable_pub',True)
      if enabled == False:
        self.tracking_enabled = False
        self.sendTargetsMsg('save_pub',True)
        nepi_sdk.sleep(1)
        self.targets_status_msg = None
        if True: #self.tracking_manages_targeting == False:
            self.sendTargetsConfig()
        nepi_sdk.sleep(1)
        
  


  def checkForTargetsTopic(self,namespace):
      check_topic = namespace + '/status'
      found = check_topic in self.active_topics
      return found


  def updaterTrackingStateCb(self,timer):
        #self.msg_if.pub_warn("Tracking Updater Called")

        needs_publish = False
 
        ####################
        # Update State
        self.tracking_state = copy.deepcopy(self.track_dict_check) is not None
        #self.msg_if.pub_warn("Resetting Tracking State")
        self.track_dict_check = None
        self.track_dict = None
        track_last_time = 0

        ##################
        # Get settings from param server
        # if needs_publish == True:
        #     self.publish_status()

        nepi_sdk.start_timer_process(self.targets_timeout, self.updaterTrackingStateCb, oneshot = True)   



  def updaterTrackingCb(self,timer):
    #self.msg_if.pub_warn("Tracking Updater Called")

    needs_publish = False
    tracking_dict = copy.deepcopy(self.tracking_dict)
    targets_topic =  tracking_dict['targets_topic']
    source_topic = tracking_dict['source_topic']
    
    track_sources = nepi_track.get_source_namespaces(topics_list = self.active_topics, types_list = self.active_topic_types)
    self.tracking_available_targets = track_sources

    # self.msg_if.pub_warn("")
    # self.msg_if.pub_warn("")
    # self.msg_if.pub_warn("")
    # self.msg_if.pub_warn("Starting Tracking Updater Purge Check")
    # self.msg_if.pub_warn("available targets: " + str(self.tracking_available_targets))
    # self.msg_if.pub_warn("tracking_dict: " + str(tracking_dict))
    # connect_states = [self.tracking_targets_connected,self.tracking_targets_connecting]
    # self.msg_if.pub_warn("connect states: " + str(connect_states))
    # cur_namespace = self.tracking_targets_connected_namespace
    # self.msg_if.pub_warn("current_namespace: " + str(cur_namespace))
    # active = self.checkForTargetsTopic(cur_namespace)
    # self.msg_if.pub_warn("is active topic: " + str(active))

    ####################
    #### Purge if needed
    do_purge = False
    if (self.tracking_targets_connected == True or self.tracking_targets_connecting == True):
        cur_namespace = self.tracking_targets_connected_namespace
        if self.checkForTargetsTopic(cur_namespace) == False and cur_namespace != 'None' and self.tracking_subpub_dict is not None:
            self.msg_if.pub_warn("Unsubscribing to Targets self.tracking_targets_connected_namespace: " + str(cur_namespace))
            success = self.unsubscribeTargets()
            needs_publish = True

    # self.msg_if.pub_warn("")
    # self.msg_if.pub_warn("Starting Tracking Updater Connect Check")
    # self.msg_if.pub_warn("available targets: " + str(self.tracking_available_targets))
    # self.msg_if.pub_warn("tracking_dict: " + str(tracking_dict))
    # connect_states = [self.tracking_targets_connected,self.tracking_targets_connecting]
    # self.msg_if.pub_warn("connect states: " + str(connect_states))
    # cur_namespace = self.tracking_targets_connected_namespace
    # self.msg_if.pub_warn("current_namespace: " + str(cur_namespace))
    # active = self.checkForTargetsTopic(cur_namespace)
    # self.msg_if.pub_warn("is active topic: " + str(active))
    # targets_topic =  tracking_dict['targets_topic']
    # self.msg_if.pub_warn("check_namespace: " + str(targets_topic))
    # active = self.checkForTargetsTopic(targets_topic)
    # self.msg_if.pub_warn("is active topic: " + str(active))


    ####################
    #### Connect if needed
    needs_connect = False
    cur_namespace = self.tracking_targets_connected_namespace
    if (targets_topic != cur_namespace and self.checkForTargetsTopic(targets_topic) and targets_topic != 'None'):
        if (self.tracking_targets_connected == False and self.tracking_targets_connecting == False):
            self.msg_if.pub_warn("Subscribing to Targets topic: " + str(targets_topic) + " does not match current namespace " + str(cur_namespace))
            needs_connect = True
            success = self.subscribeTargets(targets_topic)
            needs_publish = True
    
      

    ##################
    # Update status

    # self.msg_if.pub_warn("")
    # self.msg_if.pub_warn("Starting Tracking Status Msg Check")
    # self.msg_if.pub_warn("status_msg is None: " + str(self.targets_status_msg is None))


    if self.tracking_targets_connected == True:
        if self.targets_status_last_time is not None:
            last_time = nepi_utils.get_time() - self.targets_status_last_time
            if last_time > self.targets_timeout:
                self.msg_if.pub_warn("Clearing targets status message on timeout")
                self.targets_status_msg = None
                self.targets_status_last_time = None
                self.tracking_running = False
                self.tracking_available_sources = []
                self.tracking_available_classes = []
                needs_publish = True


    ##################
    # Get settings from param server
    if needs_publish == True:
      self.publish_status()

    nepi_sdk.start_timer_process(1.0, self.updaterTrackingCb, oneshot = True)     




  def subscribeTargets(self,namespace):
        success = True
        if self.tracking_subpub_dict is not None:
            success = self.unsubscribeTargets()
        if namespace != 'None':
            self.tracking_targets_connecting = True
            self.tracking_targets_connected_namespace = namespace
            self.msg_if.pub_warn("Subscribing to Targets topic: " + str(namespace))
            tracking_subpub_dict = dict()

            tracking_subpub_dict['targets_sub'] = nepi_sdk.create_subscriber(namespace, Targets, self.targetsCb, queue_size = 1, callback_args= (namespace), log_name_list = [])
            tracking_subpub_dict['status_sub'] = nepi_sdk.create_subscriber(namespace + '/status', TargetingStatus, self.targetsStatusCb, queue_size = 1, callback_args= (namespace), log_name_list = [])
            
            tracking_subpub_dict['enable_pub'] = nepi_sdk.create_publisher(namespace + '/enable', Bool, queue_size = 10, log_name_list = [])
            tracking_subpub_dict['source_pub'] = nepi_sdk.create_publisher(namespace + '/set_source_topic', String, queue_size = 10, log_name_list = [])
            tracking_subpub_dict['class_pub'] = nepi_sdk.create_publisher(namespace + '/set_class', String, queue_size = 10, log_name_list = [])
            tracking_subpub_dict['threshold_pub'] = nepi_sdk.create_publisher(namespace + '/set_threshold', Float32, queue_size = 10, log_name_list = [])
            tracking_subpub_dict['save_pub'] = nepi_sdk.create_publisher(namespace + '/set_save_config_enable', Bool, queue_size = 10, log_name_list = [])
            tracking_subpub_dict['config_pub'] = nepi_sdk.create_publisher(namespace + '/set_config', TargetingUpdate, queue_size = 10, log_name_list = [])

            self.tracking_subpub_lock.acquire()
            self.tracking_subpub_dict = tracking_subpub_dict
            nepi_sdk.sleep(1)
            self.tracking_subpub_lock.release()

            self.sendTargetsMsg('enable_pub',True)


            self.tracking_targets_connected = False
            
            self.targets_status_msg = None
            self.targets_status_msg_start = None  
        else:
            self.tracking_targets_connecting = False
        return success  

  def unsubscribeTargets(self):
        if self.tracking_subpub_dict is not None:
            namespace = self.tracking_targets_connected_namespace
            self.msg_if.pub_info("Unsubscribing to Targets topic: " + str(namespace))

            if True: #self.tracking_manages_targeting == False:
                self.sendTargetsConfig()
            nepi_sdk.sleep(1)
            self.sendTargetsMsg('save_pub',True)

            self.tracking_subpub_lock.acquire()
            for subpub in self.tracking_subpub_dict.keys():
                try:
                    self.tracking_subpub_dict[subpub].unregister()
                except:
                    pass
            self.tracking_subpub_lock.release()
            nepi_sdk.sleep(1)
        self.tracking_targets_connecting = False
        self.tracking_targets_connected = False
        self.tracking_targets_connected_namespace = 'None'
        self.targets_status_msg = None
        self.targets_status_msg_start = None 
        self.targets_status_last_time = None
        return True


  def customFilterCb(self,targets_dict_list):
     filtered_dict_list = self.filter_by_range_angles(targets_dict_list)
     return filtered_dict_list


  def targetsCb(self,msg, args):
    #self.msg_if.pub_info("Targets callback got new targets mgs: " + str(msg))
    targets_namespace = args
    self.targets_last_time = nepi_utils.get_time()
    #self.msg_if.pub_warn("Tracking enabled " + str(self.tracking_enabled))
    if self.tracking_enabled == True:
        self.targets_msg = msg.targets
        #self.msg_if.pub_info("Got targets mgs: " + str(self.targets_msg))
        tracking_dict = copy.deepcopy(self.tracking_dict)
        targets_topic = tracking_dict['targets_topic']
        source_topic = tracking_dict['source_topic']
        if True: #targets_topic == msg.process_namespace and source_topic == msg.source_topic:
            self.targets_last_time = nepi_utils.get_time()

            #self.msg_if.pub_warn("Got targets msg list " + str(targets_msg))
            targets_dict_list = []
            for target_msg in self.targets_msg:
                target_dict = nepi_targets.convert_target_msg2dict(target_msg)
                targets_dict_list.append(target_dict)
                #self.msg_if.pub_warn("Added target list for name " + str(target_dict['target_name']))
            #self.msg_if.pub_warn("Got targets list " + str(targets_dict_list))


            #########################
            # Apply Cutsom Filter Targets
            # if self.customFilterCb is not None:
            #     targets_dict_list = self.customFilterCb(targets_dict_list)
            
            #self.msg_if.pub_warn("Got custom filtered targets list " + str(targets_dict_list))
            #################
            # Find Best Target
            #self.msg_if.pub_warn("Applying best target filter dict " + str(tracking_dict))
            [best_target_dict,tracking_dict] = nepi_track.get_best_from_targets(targets_dict_list,tracking_dict)
            #self.msg_if.pub_warn("Got best targets " + str(best_target_dict))
            if best_target_dict is not None:
                self.track_last_time = nepi_utils.get_time()
                self.track_dict = best_target_dict
                self.track_pan_dict = copy.deepcopy(best_target_dict)
                self.track_tilt_dict = copy.deepcopy(best_target_dict)
                self.track_dict_check = best_target_dict
                ##################
                #self.msg_if.pub_warn("Got track dict" + str(track_dict))


                track_msg = nepi_targets.convert_target_dict2msg(best_target_dict)
                if self.node_if is not None:
                    self.node_if.publish_pub('track', track_msg) 

  def targetsStatusCb(self,msg, args):
    #self.msg_if.pub_info("Targets callback got new targets mgs")
    targets_namespace = args
    status_msg = msg
    namespace = status_msg.namespace
    if self.targets_status_msg_start is None:
        #self.msg_if.pub_warn("Captured current Status msg from Targers namespace: " + str(namespace) + " : " + str(msg))
        self.targets_status_msg_start = msg
    self.tracking_running = True
    self.tracking_targets_connecting = False
    self.tracking_targets_connected = True
    self.targets_status_msg = msg
    self.tracking_available_sources = msg.available_source_topics
    self.tracking_available_classes = msg.available_classes

    self.targets_status_last_time = nepi_utils.get_time()



 



##########################
### Status
##########################  

 
  

  def publishStatusCb(self,timer):


    self.publish_status()


  def publish_status(self):
    self.status_msg.available_servos = self.available_servos
    selected_servo = 'None'
    if self.selected_servo in self.available_servos:
       selected_servo = self.selected_servo
    self.status_msg.selected_servo = selected_servo

    sv_connected_topic = self.sv_connected_topic
    if sv_connected_topic is None:
       sv_connected_topic = 'None'
    self.status_msg.sv_connected_topic = sv_connected_topic
    self.status_msg.sv_connected = self.sv_connected

 
    current_position = [-999,-999]
    #self.msg_if.pub_warn("self.current_position: " + str(self.current_position))
    if self.current_position is not None:
      current_position = self.current_position

    sv_status_msg = None
    if self.sv_connect_if is not None:
       sv_status_msg = self.sv_connect_if.get_status_msg()
    
    if sv_status_msg is None:
       sv_status_msg = DeviceSVXStatus()
    
    
    self.status_msg.pan_deg = sv_status_msg.pan_now_deg
    self.status_msg.tilt_deg = sv_status_msg.tilt_now_deg

    self.status_msg.pan_goal = sv_status_msg.pan_goal_deg
    self.status_msg.tilt_goal = sv_status_msg.tilt_goal_deg


    self.status_msg.sv_status_msg = sv_status_msg
    self.servo_max_speed_dps = sv_status_msg.speed_max_dps
    self.status_msg.servo_max_speed_dps = sv_status_msg.speed_max_dps
    self.pan_deg_per_sec = sv_status_msg.speed_pan_dps
    self.status_msg.pan_deg_per_sec = sv_status_msg.speed_pan_dps
    self.tilt_deg_per_sec = sv_status_msg.speed_tilt_dps
    self.status_msg.tilt_deg_per_sec = sv_status_msg.speed_tilt_dps
    
    self.status_msg.servo_avg_move_delay = self.servo_avg_move_delay

    self.status_msg.speed_ratio = sv_status_msg.speed_ratio
    self.status_msg.pan_speed_ratio = sv_status_msg.speed_pan_ratio
    self.status_msg.tilt_speed_ratio = sv_status_msg.speed_tilt_ratio

    ###


    self.status_msg.pan_scanning = self.pan_scanning
    self.status_msg.tilt_scanning = self.tilt_scanning

    self.status_msg.pan_tracking = self.pan_tracking
    self.status_msg.tilt_tracking = self.tilt_tracking
    self.status_msg.track_source_connected = self.targets_status_msg is not None and self.tracking_targets_connected == True

    self.status_msg.pan_stabing = self.pan_stabing
    self.status_msg.tilt_stabing = self.tilt_stabing

    goto_pos = copy.deepcopy(self.goto_position)
    self.status_msg.pan_goto = goto_pos[0]
    self.status_msg.tilt_goto = goto_pos[1]

    self.status_msg.pan_control_disabled = self.pan_scanning or self.pan_tracking or self.pan_stabing
    self.status_msg.tilt_control_disabled = self.tilt_scanning or self.tilt_tracking or self.tilt_stabing


    self.status_msg.scan_pan_enabled = self.scan_pan_enabled
    self.status_msg.scan_tilt_enabled = self.scan_tilt_enabled
    self.status_msg.scan_pan_speed_ratio = self.scan_pan_speed_ratio
    self.status_msg.scan_tilt_speed_ratio = self.scan_tilt_speed_ratio
    self.status_msg.scan_pan_min_deg = round(self.scan_pan_min_deg, 0)
    self.status_msg.scan_pan_max_deg = round(self.scan_pan_max_deg, 0)
    self.status_msg.scan_tilt_min_deg = round(self.scan_tilt_min_deg, 0)
    self.status_msg.scan_tilt_max_deg = round(self.scan_tilt_max_deg, 0)


    ###################################

    self.status_msg.track_pan_enabled = self.track_pan_enabled
    self.status_msg.track_tilt_enabled = self.track_tilt_enabled

    self.status_msg.track_move_deg = self.track_move_deg
    self.status_msg.track_goal_deg = self.track_goal_deg

    self.status_msg.track_move_ratio = self.track_move_ratio
    self.status_msg.track_reset_time_sec = self.track_reset_time_sec
    self.status_msg.track_pan_error = self.track_pan_error
    self.status_msg.track_tilt_error = self.track_tilt_error


    ###################################
    available_stab_source_dict = copy.deepcopy(self.available_stab_source_dict)
    available_stab_source_namespaces =  list(available_stab_source_dict.keys())
    self.status_msg.available_stab_source_namespaces = available_stab_source_namespaces
    selected_stab_source = self.selected_stab_source
    if selected_stab_source not in available_stab_source_namespaces:
        selected_stab_source = 'None'

    self.status_msg.selected_stab_source = selected_stab_source
    self.status_msg.stab_source_connected = self.stab_source_connected

    available_stab_processes =  self.available_stab_processes
    self.status_msg.available_stab_processes = available_stab_processes
    selected_stab_process = self.selected_stab_process
    if selected_stab_process not in available_stab_processes:
        selected_stab_process = 'None'


    self.status_msg.selected_stab_process = selected_stab_process
    self.status_msg.stab_process_ready = self.stab_process_ready
    
    self.status_msg.stab_pan_ready = self.stab_pan_ready
    self.status_msg.stab_tilt_ready = self.stab_tilt_ready

    self.status_msg.stab_pan_enabled = self.stab_pan_enabled
    self.status_msg.stab_tilt_enabled = self.stab_tilt_enabled

    stab_pos = copy.deepcopy(self.stab_position)
    self.status_msg.stab_pan_pos = stab_pos[0]
    self.status_msg.stab_tilt_pos = stab_pos[1]

    stab_settings_dict = copy.deepcopy(self.stab_processes_dict[self.selected_stab_process])
    self.status_msg.stab_update_rate = stab_settings_dict['stab_update_rate']
    self.status_msg.stab_num_avg = self.stab_processes_dict[self.selected_stab_process]['stab_num_avg']
    self.status_msg.stab_reset_time_sec = stab_settings_dict['stab_reset_time_sec']

    max_speed = self.servo_max_speed_dps
    self.status_msg.stab_sv_min_speed_ratio = stab_settings_dict['stab_sv_min_speed_ratio']
    stab_sv_max_speed_ratio = stab_settings_dict['stab_sv_max_speed_ratio']
    stab_sv_max_speed_dps = 0.0
    if max_speed != -999:
        stab_sv_max_speed_dps = max_speed * stab_sv_max_speed_ratio
    self.status_msg.stab_sv_max_speed_ratio = stab_sv_max_speed_ratio
    self.status_msg.stab_sv_max_speed_dps = stab_sv_max_speed_dps

    stab_control_names = []
    stab_control_values = []
    stab_controls_dict = stab_settings_dict['stab_controls_dict']
    for control in stab_controls_dict.keys():
        stab_control_names.append(control)
        stab_control_values.append(stab_controls_dict[control])
    self.status_msg.stab_control_names = stab_control_names
    self.status_msg.stab_control_values = stab_control_values


    self.status_msg.stab_roll_deg = self.stab_data_dict['roll_deg']
    self.status_msg.stab_roll_dps = self.stab_data_dict['roll_dps']

    self.status_msg.stab_pitch_deg = self.stab_data_dict['pitch_deg']
    self.status_msg.stab_pitch_dps = self.stab_data_dict['pitch_dps']

    self.status_msg.stab_heading_deg = self.stab_data_dict['heading_deg']
    self.status_msg.stab_heading_dps = self.stab_data_dict['heading_dps']


    self.status_msg.stab_pan_deg = self.stab_data_dict['stab_pan_deg']
    self.status_msg.stab_pan_dps = self.stab_data_dict['stab_pan_dps']
    self.status_msg.stab_pan_adj = self.stab_data_dict['stab_pan_adj']
    self.status_msg.stab_pan_goal = self.stab_data_dict['stab_pan_goal']
    self.status_msg.stab_pan_pos_rate = self.stab_data_dict['stab_pan_pos_rate']
    self.status_msg.stab_pan_vel_rate = self.stab_data_dict['stab_pan_vel_rate']
    

    self.status_msg.stab_tilt_deg = self.stab_data_dict['stab_tilt_deg']
    self.status_msg.stab_tilt_dps = self.stab_data_dict['stab_tilt_dps']
    self.status_msg.stab_tilt_adj = self.stab_data_dict['stab_tilt_adj']
    self.status_msg.stab_tilt_goal = self.stab_data_dict['stab_tilt_goal']
    self.status_msg.stab_tilt_pos_rate = self.stab_data_dict['stab_tilt_pos_rate']
    self.status_msg.stab_tilt_vel_rate = self.stab_data_dict['stab_tilt_vel_rate']
    


    ###############
    ###Image Viewer
    ###############
    image_topics = copy.deepcopy(self.selected_image_topics)
    self.status_msg.num_windows = self.num_windows
    if self.num_windows == 1:
        image_topics[0] = self.single_image_topic
    for i, topic in enumerate(image_topics):
       if topic != 'None':
          if topic not in self.available_image_topics:
             image_topics[i] = 'None'
    self.status_msg.image_topics = image_topics
    self.status_msg.image_priority_options = self.image_priority_list

    ############

    ###############
    # Tracking
    ###############
    # ################
    # # Targeting Status
    targets_status_msg = copy.deepcopy(self.targets_status_msg)
    tracking_dict = copy.deepcopy(self.tracking_dict)

    self.tracking_status_msg.name = self.node_name
    self.tracking_status_msg.node_name = self.node_name
    self.tracking_status_msg.namespace = self.node_namespace

    #self.tracking_status_msg.save_data_topic = self.save_data_namespace

    #################
    self.tracking_status_msg.enabled = self.tracking_enabled
    self.tracking_status_msg.running = self.tracking_running
    self.tracking_status_msg.state = self.tracking_state

    ##############
    self.tracking_status_msg.manages_targeting = self.tracking_manages_targeting

    targets_sources = copy.deepcopy(self.tracking_available_targets)
    self.tracking_status_msg.available_targets_topics = targets_sources
    targets_topic = tracking_dict['targets_topic']
    #self.msg_if.pub_warn("Status checking target against targets: " + str(targets_topic) + " " + str(targets_sources))
    if targets_topic not in targets_sources:
       #self.msg_if.pub_warn("Status did not find target")
       targets_topic = 'None'
    self.tracking_status_msg.selected_targets = targets_topic
    targets_connected = targets_status_msg is not None and self.tracking_targets_connected == True

    ##############
    threshold = tracking_dict['threshold_filter']
    self.tracking_status_msg.threshold_filter = threshold
    

    self.tracking_status_msg.size_min_filter = tracking_dict['size_min_filter']
    self.tracking_status_msg.size_max_filter = tracking_dict['size_max_filter']


    self.tracking_status_msg.range_filter_min = tracking_dict['range_min_filter']
    self.tracking_status_msg.range_filter_max = tracking_dict['range_max_filter']

    self.tracking_status_msg.available_best_filters = self.tracking_best_filter_options
    self.tracking_status_msg.selected_best_filter  = tracking_dict['best_filter']


    ################
    self.tracking_status_msg.targets_connected = targets_connected

    self.tracking_status_msg.available_source_topics = []
    self.tracking_status_msg.selected_source = 'None'
    self.tracking_status_msg.available_classes = []
    self.tracking_status_msg.selected_class = 'None'

    if targets_status_msg is not None and targets_connected == True:

        if self.tracking_enabled == True and targets_status_msg.enabled == False:
            self.sendTargetsMsg('enable_pub',True)

        track_sources = targets_status_msg.available_source_topics
        self.tracking_status_msg.available_source_topics = track_sources
        track_sources_selected = targets_status_msg.selected_sources
        source_topic = tracking_dict['source_topic']
        if source_topic not in track_sources:
            source_topic = 'None'
        self.tracking_status_msg.selected_source = source_topic
        #self.msg_if.pub_warn("Status checking source against sources: " + str([source_topic]) + " " + str(track_sources_selected))
        if source_topic != 'None':
            if source_topic in track_sources_selected:
                source_ind = track_sources_selected.index(source_topic)
                if source_ind != -1:
                    try:
                        self.tracking_status_msg.source_connected = targets_status_msg.sources_connected[source_ind]
                        self.tracking_status_msg.source_have_range = targets_status_msg.sources_have_range[source_ind]
                        self.tracking_status_msg.source_fov_horz_degs = targets_status_msg.sources_fov_horz_degs[source_ind]
                        self.tracking_status_msg.source_fov_vert_degs = targets_status_msg.sources_fov_vert_degs[source_ind]
                    except:
                        pass
            if [source_topic] != track_sources_selected:
                if self.tracking_subpub_dict is not None and self.tracking_enabled == True:
                        #self.msg_if.pub_warn("Status sending source update: " + str([source_topic]))
                        self.sendTargetsMsg('source_pub',source_topic)

                
        ##################
        # track_classes = targets_status_msg.available_classes
        # self.tracking_status_msg.available_classes = track_classes
        # classes_selected = targets_status_msg.selected_classes
        # class_filter = tracking_dict['class_filter']
        # if class_filter not in track_classes:
        #     class_filter = 'None'
        # self.tracking_status_msg.selected_class = class_filter
        # if class_filter != 'None' and [class_filter] != classes_selected and self.tracking_enabled == True:
        #     #self.msg_if.pub_warn("Status sending class update: " + str([class_filter]))
        #     self.sendTargetsMsg('class_pub',class_filter)

        #################
        if round(threshold,2) != round(self.targets_status_msg.threshold_filter,2) and self.tracking_enabled == True:
            #self.msg_if.pub_warn("Status sending threshold update: " + str(threshold))
            self.sendTargetsMsg('threshold_pub',threshold)

    self.status_msg.tracking_status = self.tracking_status_msg

    ###########
    if self.node_if is not None:
      if self.status_has_published == False:
        #self.msg_if.pub_warn("Publishing Status: " + str(self.status_msg))
        self.status_has_published = True
      self.node_if.publish_pub('status_pub', self.status_msg) 
      #self.node_if.save_config()
    
  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    self.msg_if.pub_info(" Shutting down: Executing script cleanup actions")





##########################
### Stab
##########################  
  def setStabPanCb(self, msg):
        enabled = msg.data
        self.msg_if.pub_info("Setting stab pan: " + str(enabled))
        self.setStabPan(enabled)


  def setStabPan(self,enabled):
        if enabled == True and self.scan_pan_enabled == True:
            self.stab_position[0] = 0
        self.scan_pan_enabled = False
        self.track_pan_enabled = False
        self.stab_pan_enabled = enabled
        self.publish_status()

        if enabled == False and self.sv_connect_if is not None:
            self.sv_connect_if.set_pan_speed_ratio(1)
            self.sv_connect_if.goto_to_pan_position(self.stab_position[0])
        if self.node_if is not None:
            self.node_if.set_param('stab_pan_enabled', self.stab_pan_enabled)

  def setStabTiltCb(self, msg):
        enabled = msg.data
        self.msg_if.pub_info("Setting stab tilt: " + str(enabled))
        self.setStabTilt(enabled)

  def setStabTilt(self,enabled):
        if enabled == True and self.scan_tilt_enabled == True:
            self.stab_position[1] = 0
        self.scan_tilt_enabled = False
        self.track_tilt_enabled = False
        self.stab_tilt_enabled = enabled
        self.publish_status()
        if enabled == False and self.sv_connect_if is not None:
            self.sv_connect_if.set_tilt_speed_ratio(1)
            self.sv_connect_if.goto_to_tilt_position(self.stab_position[1])
        if self.node_if is not None:
            self.node_if.set_param('stab_tilt_enabled', self.stab_tilt_enabled)


  def setStabResetTimeSecCb(self, msg):
      reset_time = msg.data
      self.setStabResetTimeSec(reset_time)

  def setStabResetTimeSec(self,reset_time):
        if reset_time < 1:
            reset_time = 1
        if reset_time > self.STAB_MAX_RESET_SEC:
            reset_time = self.STAB_MAX_RESET_SEC
        self.msg_if.pub_info("Setting stab reset time to: " + str(reset_time))
        self.stab_processes_dict[self.selected_stab_process]['stab_reset_time_sec'] = reset_time
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('stab_processes_dict', self.stab_processes_dict)
            #self.node_if.save_config()

  def setStabSpeedRatiosCb(self, msg):
      min_ratio = msg.start_range
      max_ratio = msg.stop_range
      self.setStabSpeedRatios(min_ratio,max_ratio)


  def setStabSpeedRatios(self,min_ratio,max_ratio):
        min_ratio = nepi_utils.check_ratio(min_ratio)
        max_ratio = nepi_utils.check_ratio(max_ratio)
        if min_ratio > max_ratio:
            self.msg_if.pub_info("invalid speed ratios: " + "%.2f" % min_ratio  + " " + "%.2f" % max_ratio )
        else:
            #self.msg_if.pub_info("Setting stab speed ratio limits to: " + "%.2f" % adj_min_deg  + " " + "%.2f" % adj_max_deg )
            self.msg_if.pub_info("Setting stab max speed ratios to: " + str([min_ratio,max_ratio]))
            self.stab_processes_dict[self.selected_stab_process]['stab_sv_min_speed_ratio'] = min_ratio
            self.stab_processes_dict[self.selected_stab_process]['stab_sv_max_speed_ratio'] = max_ratio
            self.publish_status()
            if self.node_if is not None:
                self.node_if.set_param('stab_processes_dict', self.stab_processes_dict)
                #self.node_if.save_config()

  def setStabControlCb(self, msg):
      self.msg_if.pub_info("Got Stab Control update message " + str(msg))
      control = msg.name
      value = msg.value
      self.setStabControl(control,value)

  def setStabControl(self, control,value):
        stab_process = self.selected_stab_process
        stab_controls_dict = self.stab_processes_dict[stab_process]['stab_controls_dict']
        if control in stab_controls_dict.keys():
            self.msg_if.pub_info("Setting stab control " + str(control) + " : " + str(value))
            stab_controls_dict[control] = value
            self.stab_processes_dict[stab_process]['stab_controls_dict'] = stab_controls_dict
            self.publish_status()
            if self.node_if is not None:
                self.node_if.set_param('stab_processes_dict', self.stab_processes_dict)
                #self.node_if.save_config()



  def setStabPanPosRatioCb(self, msg):
      self.setStabPanPosRatio(msg.data)

  def setStabPanPosRatio(self, speed_ratio):
        speed_ratio = nepi_utils.check_ratio(speed_ratio)
        if speed_ratio < 0.1:
            speed_ratio = 0.1
        self.stab_position[0] = self.min_pan_softstop_deg + speed_ratio * (self.max_pan_softstop_deg - self.min_pan_softstop_deg)

  def setStabTiltPosRatioCb(self, msg):
      self.setStabTiltPosRatio(msg.data)

  def setStabTiltPosRatio(self, speed_ratio):
        speed_ratio = nepi_utils.check_ratio(speed_ratio)
        if speed_ratio < 0.1:
            speed_ratio = 0.1
        speed_ratio = 1 - speed_ratio
        self.stab_position[1] = self.min_tilt_softstop_deg + speed_ratio * (self.max_tilt_softstop_deg - self.min_tilt_softstop_deg)

  def setStabUpdateRateCb(self, msg):
      rate = msg.data
      self.setStabUpdateRate(rate)

  def setStabUpdateRate(self, rate):
        if rate < 0:
            rate = 1
        rate = round(rate,1)
        self.msg_if.pub_info("Setting stab update rate to: " + str(rate))
        self.stab_processes_dict[self.selected_stab_process]['stab_update_rate'] = rate
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('stab_processes_dict', self.stab_processes_dict)
            #self.node_if.save_config()

  def setStabNumAvgCb(self, msg):
      num_avg = msg.data
      self.setStabNumAvg(num_avg)

  def setStabNumAvg(self, num_avg):
        if num_avg < 1:
            num_avg = 1
        self.msg_if.pub_info("Setting stab num avg to: " + str(num_avg))
        self.stab_processes_dict[self.selected_stab_process]['stab_num_avg'] = num_avg
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('stab_processes_dict', self.stab_processes_dict)
            #self.node_if.save_config()


  def reloadStabsCb(self,msg):
    self.stab_process_ready = False
    nepi_sdk.sleep(1)
    try:
        importlib.reload(nepi_stab_sv)
        self.stab_processes_dict = nepi_stab_sv.update_processes_dict(self.stab_processes_dict)
        stab_processes = list(self.stab_processes_dict.keys())
        if self.selected_stab_process not in stab_processes:
            self.selected_stab_process = stab_processes[0]
        self.msg_if.pub_info("Stabs reloaded")
        self.stab_process_ready = True
    except Exception as e:
        self.msg_if.pub_info("Failed to reload stab module: " + str(e)) 


  def setStabSourceCb(self, msg):
      value = msg.data
      self.setStabSource(value)

  def setStabSource(self,value):
        self.msg_if.pub_info("Setting stab source topic to: " + str(value))
        self.selected_stab_source = value
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('selected_stab_source', self.selected_stab_source)
            #self.node_if.save_config()


  def setStabProcessCb(self, msg):
      value = msg.data
      self.setStabProcess(value)

  def setStabProcess(self,value):
        self.msg_if.pub_info("Setting stab process topic to: " + str(value))
        if value in self.stab_processes_dict.keys():
            self.selected_stab_process = value
            self.publish_status()
            if self.node_if is not None:
                self.node_if.set_param('selected_stab_process', self.selected_stab_process)
                #self.node_if.save_config()


  def checkForStabSourceTopic(self,namespace):
      check_topic = namespace
      found = check_topic in list(self.available_stab_source_dict.keys())
      return found

  def updaterStabCb(self,timer):
    #self.msg_if.pub_warn("Stab Updater Called")

    needs_publish = False
    avail_stab_sources = []
    avail_stab_sources_dict = dict()
    for message in nepi_stab_sv.SOURCE_MESSAGE_DICT.keys():
        avail_sources = nepi_sdk.find_topics_by_msg(message,self.active_topics,self.active_topic_types)
        for source in avail_sources:
            if message != 'NavPose' or (message == 'NavPose' and 'navposes' in source and os.path.basename(source) == 'navpose'):
                avail_stab_sources.append(source)
                avail_stab_sources_dict[source] = nepi_stab_sv.SOURCE_MESSAGE_DICT[message]              
    self.available_stab_source_dict = avail_stab_sources_dict
    source_topic = self.selected_stab_source

    

    # self.msg_if.pub_warn("")
    # self.msg_if.pub_warn("")
    # self.msg_if.pub_warn("")
    # self.msg_if.pub_warn("Starting Stab Updater Purge Check")
    # connect_states = [self.stab_source_connected,self.stab_source_connecting]
    # self.msg_if.pub_warn("connect states: " + str(connect_states))
    # cur_namespace = self.stab_source_connected_namespace
    # self.msg_if.pub_warn("current_namespace: " + str(cur_namespace))
    # active = self.checkForStabSourceTopic(cur_namespace)
    # self.msg_if.pub_warn("is active topic: " + str(active))

    ####################
    #### Purge if needed
    do_purge = False
    if (self.stab_source_connected == True or self.stab_source_connecting == True):
        cur_namespace = self.stab_source_connected_namespace
        if source_topic not in avail_stab_sources and cur_namespace != 'None' and self.stab_subpub_dict is not None:
            self.msg_if.pub_warn("Unsubscribing to Stab self.stab_source_connected_namespace: " + str(cur_namespace))
            success = self.unsubscribeStabSource()
            needs_publish = True

    # self.msg_if.pub_warn("")
    # self.msg_if.pub_warn("Starting Stab Updater Connect Check")
    # connect_states = [self.stab_source_connected,self.stab_source_connecting]
    # self.msg_if.pub_warn("connect states: " + str(connect_states))
    # cur_namespace = self.stab_source_connected_namespace
    # self.msg_if.pub_warn("current_namespace: " + str(cur_namespace))
    # active = self.checkForStabSourceTopic(cur_namespace)
    # self.msg_if.pub_warn("is active topic: " + str(active))
    # self.msg_if.pub_warn("check_namespace: " + str(source_topic))
    # active = self.checkForStabSourceTopic(source_topic)
    # self.msg_if.pub_warn("is active topic: " + str(active))


    ####################
    #### Connect if needed
    needs_connect = False
    if source_topic == 'None':
        for source in avail_sources:
            if self.STAB_DEFAULT_SOURCE in source:
                source_topic = source
                self.selected_stab_source = source_topic

        
    cur_namespace = self.stab_source_connected_namespace
    if (source_topic != cur_namespace and source_topic in avail_stab_sources and source_topic != 'None'):
        if (self.stab_source_connected == False and self.stab_source_connecting == False):
            self.msg_if.pub_warn("Subscribing to Stab topic: " + str(source_topic) + " does not match current namespace " + str(cur_namespace))
            needs_connect = True
            success = self.subscribeStabSource(source_topic)
            needs_publish = True
    
      

    ##################
    # Update status

    # self.msg_if.pub_warn("")
    # self.msg_if.pub_warn("Starting Stab Status Msg Check")
    # self.msg_if.pub_warn("status_msg is None: " + str(self.stab_status_msg is None))

    stab_reset_time_sec = self.stab_processes_dict[self.selected_stab_process]['stab_reset_time_sec']  
    if self.stab_source_connected == True:
        if self.last_stab_source_time is not None:
            last_time = nepi_utils.get_time() - self.last_stab_source_time
            if last_time > stab_reset_time_sec:
                self.msg_if.pub_warn("Clearing stab status message on timeout")

                self.stab_dict_lock.acquire()
                self.stab_data_dict = nepi_stab_sv.get_blank_data_dict()
                self.stab_dict_lock.release()  
                self.stab_pan_adj = 0.0
                self.stab_tilt_adj = 0.0
                self.last_stab_source_time = None
                self.stab_running = False

                needs_publish = True

    ####################
    # Update State
    # self.stab_state = self.track_dict_check is not None
    # self.track_dict_check = None
    # self.track_dict = None
    track_last_time = 0

    ##################
    # Get settings from param server
    # if needs_publish == True:
    #   self.publish_status()

    nepi_sdk.start_timer_process(1.0, self.updaterStabCb, oneshot = True)     



  def subscribeStabSource(self,namespace):
        success = True
        if self.stab_subpub_dict is not None:
            success = self.unsubscribeStabSource()
        if namespace != 'None' and namespace in self.available_stab_source_dict.keys():
            self.stab_source_connecting = True
            self.stab_source_connected_namespace = namespace
            self.msg_if.pub_warn("Subscribing to Stab topic: " + str(namespace))
            stab_subpub_dict = dict()

            message = self.available_stab_source_dict[namespace]
            stab_subpub_dict['source_sub'] = nepi_sdk.create_subscriber(namespace, message, self.stabSourceCb, queue_size = 1, log_name_list = [])

            self.stab_subpub_lock.acquire()
            self.stab_subpub_dict = stab_subpub_dict
            nepi_sdk.sleep(1)
            self.stab_subpub_lock.release()

            self.stab_source_connected = False

        else:
            self.stab_source_connecting = False
        return success  

  def unsubscribeStabSource(self):
        if self.stab_subpub_dict is not None:
            namespace = self.stab_source_connected_namespace
            self.msg_if.pub_info("Unsubscribing to Stab topic: " + str(namespace))

            self.stab_subpub_lock.acquire()
            for subpub in self.stab_subpub_dict.keys():
                try:
                    self.stab_subpub_dict[subpub].unregister()
                except:
                    pass
            self.stab_subpub_lock.release()
            nepi_sdk.sleep(1)
        self.stab_source_connecting = False
        self.stab_source_connected = False
        self.stab_source_connected_namespace = 'None'
        self.last_stab_source_time = None
        return True


  def goto_to_pan_position_adj(self,pan_deg):
    if self.stab_pan_enabled == True and self.pan_track_hold == False:
        adj_pan_goal = pan_deg + self.stab_pan_adj
        self.sv_connect_if.goto_to_pan_position(adj_pan_goal)

  def goto_to_tilt_position_adj(self,tilt_deg):
    if self.stab_tilt_enabled == True and self.tilt_track_hold == False:
        adj_tilt_goal = tilt_deg + self.stab_tilt_adj
        self.sv_connect_if.goto_to_tilt_position(adj_tilt_goal)



  def stabSourceCb(self,msg):
    #self.msg_if.pub_warn("******")
    #self.msg_if.pub_warn("*** Stabs Source Update Starting ***")
    #self.msg_if.pub_warn("******", throttle_s=1)
    self.stab_source_connecting = False
    self.stab_source_connected = True
    source_dict = nepi_sdk.convert_msg2dict(msg)
    self.stab_source_lock.acquire()
    self.stab_source_dict = source_dict
    self.stab_source_lock.release()
    last_stab_source_time = copy.deepcopy(self.last_stab_source_time)
    if last_stab_source_time is None:
        last_stab_source_time = 0
    cur_time = nepi_utils.get_time()
    self.last_stab_source_time = cur_time


  def updaterStabSolutionCb(self, timer):
    #self.msg_if.pub_warn("******")
    #self.msg_if.pub_warn("*** Stabs Process Update Starting ***")
    #self.msg_if.pub_warn("******")


    
    start_time = nepi_utils.get_time()

    selected_stab_process = copy.deepcopy(self.selected_stab_process)
    stab_settings_dict = copy.deepcopy(self.stab_processes_dict[selected_stab_process])

    self.stab_dict_lock.acquire()
    stab_data_dict_last = copy.deepcopy(self.stab_data_dict)
    stab_data_dict = copy.deepcopy(self.stab_data_dict)
    self.stab_dict_lock.release()

    self.stab_source_lock.acquire()
    source_dict = self.stab_source_dict
    self.stab_source_lock.release()

    sv_status_msg = None
    [pan_deg,tilt_deg] = [0,0]
    if self.sv_connect_if is not None:
        sv_status_msg = self.sv_connect_if.get_status_msg()
        if sv_status_msg is not None:
            [pan_deg,tilt_deg] = self.sv_connect_if.get_servo_position()
            stab_data_dict['pan_deg'] = pan_deg
            stab_data_dict['tilt_deg'] = tilt_deg 


    # Update stab goals if disabled
    if self.stab_pan_enabled == False:
        self.stab_position[0] = pan_deg
    if self.stab_tilt_enabled == False:
        self.stab_position[1] = tilt_deg


    if source_dict is not None:

        #############################
        # Update Time Data
        #############################
        cur_time = nepi_utils.get_time()
        stab_data_dict['data_time'] = cur_time
        stab_data_dict['process_time'] = cur_time
        last_time = copy.deepcopy(stab_data_dict['last_stab_time'])
                    
        if last_time is None or stab_data_dict_last is None:
                self.msg_if.pub_warn("Stab process got bad time: " + str(last_time) )
                self.msg_if.pub_warn("Stab process got bad data: " + str(stab_data_dict_last) )
                pass
        else:
            delta_time = nepi_utils.get_time() - last_time
            

            #############################
            # Update Servo Data
            #############################

            if sv_status_msg is not None:
                servo_max_speed_dps = sv_status_msg.speed_max_dps
                stab_data_dict['servo_max_speed_dps'] = servo_max_speed_dps
                stab_data_dict['pan_dps'] = sv_status_msg.speed_pan_dps
                stab_data_dict['tilt_dps'] = sv_status_msg.speed_tilt_dps
            
            stab_data_dict['pan_speed_start_ratio'] = self.stab_pan_speed_start
            stab_data_dict['tilt_speed_start_ratio'] = self.stab_tilt_speed_start

            stab_data_dict['pan_min_deg'] = self.scan_pan_min_deg     
            stab_data_dict['pan_max_deg'] = self.scan_pan_max_deg
            stab_data_dict['tilt_min_deg'] = self.scan_tilt_min_deg
            stab_data_dict['tilt_max_deg'] = self.scan_tilt_max_deg


            self.stab_dict_lock.acquire()
            self.stab_data_dict = stab_data_dict
            self.stab_dict_lock.release()


            #############################
            # Update Nav Data
            #############################
            num_avg = stab_settings_dict['stab_num_avg']

            [roll_deg,pitch_deg] = [source_dict['roll_deg'],source_dict['pitch_deg']]
            stab_data_dict['roll_deg'] = roll_deg
            stab_data_dict['pitch_deg'] = pitch_deg

            roll_dps = (stab_data_dict['roll_deg'] - stab_data_dict_last['roll_deg']) / delta_time
            rolls_dps = stab_data_dict['rolls_dps']
            if (len(rolls_dps) >= num_avg):
                rolls_dps.pop(0)
            rolls_dps.append(roll_dps)
            stab_data_dict['roll_dps'] =  sum(rolls_dps) / len(rolls_dps)

            pitch_dps = (stab_data_dict['pitch_deg'] - stab_data_dict_last['pitch_deg']) / delta_time
            pitchs_dps = stab_data_dict['pitchs_dps']
            if (len(pitchs_dps) >= num_avg):
                pitchs_dps.pop(0)
            pitchs_dps.append(pitch_dps)
            stab_data_dict['pitch_dps'] =  sum(pitchs_dps) / len(pitchs_dps)

            stab_pan_ready = False
            heading_deg = 0.0
            heading_dps = 0.0
            if 'heading_deg' in source_dict.keys():
                if source_dict['heading_deg'] != -999:
                    stab_pan_ready = True
                    heading_deg = source_dict['heading_deg']
                    heading_dps = (stab_data_dict['heading_deg'] - stab_data_dict_last['heading_deg']) / delta_time
                    headings_dps = stab_data_dict['headings_dps']
                    if (len(headings_dps) >= num_avg):
                        headings_dps.pop(0)
                    headings_dps.append(heading_dps)
                    heading_dps = sum(headings_dps) / len(headings_dps)
            
            self.stab_pan_ready = stab_pan_ready
            stab_data_dict['heading_deg'] = heading_deg
            stab_data_dict['heading_dps'] = heading_dps

            ##########################
            # Calculate servo adjustments
            ##########################
            ## Transpose Source Frame Nav to Servo Frame
            stab_tilt_ready = False
            rpy_vector = [roll_deg, -1 * pitch_deg, heading_deg ]
            if -999 not in rpy_vector:
                stab_tilt_ready = True
                [ar,ap,ay] = rpy_vector
                #self.msg_if.pub_warn("Stabs rpy input: " + str([ar,ap,ay]))
                [art,apt,ayt]  = nepi_nav.rotate_enu_angles([ar,ap,ay],tilt_deg,'y')
                [ar,ap,ay] = [art,apt,ayt]
                #self.msg_if.pub_warn("Stabs rpy at tilt: " + str(tilt_deg) + " : " + str([ar,ap,ay]))
                [arp,app,ayp]  = nepi_nav.rotate_enu_angles([ar,ap,ay],pan_deg,'z')
                [ar,ap,ay] = [arp,app,ayp]
                #self.msg_if.pub_warn("Stabs rpy at pan: " + str(pan_deg) + " : " + str([ar,ap,ay]))

                
                ## Calculate Pan Adjustment
                p_adj = 0 #####
                pan_adjs = stab_data_dict['stab_pan_adjs']
                if (len(pan_adjs) >= num_avg):
                    pan_adjs.pop(0)
                pan_adjs.append(p_adj)
                pan_adj =  sum(pan_adjs) / len(pan_adjs)
                stab_data_dict['stab_pan_adjs'] = pan_adjs
                stab_data_dict['stab_pan_adj'] = pan_adj                
                self.stab_pan_adj = pan_adj


                ## Calculate Tilt Adjustment
                t_adj = ap
                tilt_adjs = stab_data_dict['stab_tilt_adjs']
                if (len(tilt_adjs) >= num_avg):
                    tilt_adjs.pop(0)
                tilt_adjs.append(t_adj)
                tilt_adj =  sum(tilt_adjs) / len(tilt_adjs)

                stab_data_dict['stab_tilt_adjs'] = tilt_adjs 
                stab_data_dict['stab_tilt_adj'] = tilt_adj  
                self.stab_tilt_adj = tilt_adj

                #self.msg_if.pub_warn("Stabs servo adj: " + str([pan_adj,tilt_adj]))
            self.stab_tilt_ready = stab_tilt_ready

            ##########################
            # Run Stab Process
            ##########################
            if self.stab_process_ready == True and self.sv_connected == True and self.servo_max_speed_dps != -999:
                # self.stab_count += 1
                # self.msg_if.pub_warn("Stab_count: " + str(self.stab_count))                 

                stab_pan_enabled = self.stab_pan_enabled == True and self.pan_track_hold == False
                stab_tilt_enabled = self.stab_tilt_enabled == True and self.tilt_track_hold == False


                self.pan_stabing = stab_pan_enabled and self.stab_pan_ready
                self.tilt_stabing = stab_tilt_enabled and self.stab_tilt_ready
                if selected_stab_process in nepi_stab_sv.PROCESSES_DICT.keys():
                    stab_process_function = nepi_stab_sv.PROCESSES_DICT[selected_stab_process]['process_function']
                    #self.msg_if.pub_warn("Stabs calling process function: " + str(stab_process_function), throttle_s=1)
                    [stab_data_dict, stab_settings_dict] = stab_process_function(self.sv_connect_if, 
                                                                                        stab_data_dict, 
                                                                                        stab_settings_dict, 
                                                                                        self.stab_position,
                                                                                        stab_pan_enabled, 
                                                                                        stab_tilt_enabled)
                    
    ##########################
    # Update stab settings and data dictionaries                                                                    
    #self.msg_if.pub_warn("Stabs update process complete", throttle_s=1)           
    stab_data_dict['last_stab_time'] = nepi_utils.get_time()
    self.stab_processes_dict[selected_stab_process] = stab_settings_dict
    self.stab_dict_lock.acquire()
    self.stab_data_dict = stab_data_dict
    self.stab_dict_lock.release()

    ##########################
    # Setup next process time  
    stop_time = nepi_utils.get_time()
    stab_update_rate = stab_settings_dict['stab_update_rate']
    stab_update_time = float(1)/stab_update_rate - (stop_time - start_time)
    if stab_update_time < 0.01:
        stab_update_time = 0.01

    nepi_sdk.start_timer_process(stab_update_time, self.updaterStabSolutionCb, oneshot = True) 





#########################################
# Main
#########################################
if __name__ == '__main__':
  NepiServoAutoApp()
