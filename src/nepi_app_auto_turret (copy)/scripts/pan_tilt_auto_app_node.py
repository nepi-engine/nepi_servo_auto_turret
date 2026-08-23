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


from nepi_sdk import nepi_auto_pt

from nepi_app_pan_tilt_auto.msg import PanTiltAutoAppStatus
from nepi_interfaces.msg import DevicePTXStatus, ImageMouseEvent, MgrSystemStatus
from nepi_interfaces.msg import Target, Targets, TargetingStatus
from nepi_interfaces.msg import Track, TrackingStatus
from nepi_interfaces.msg import NavPose, NavPoseOrientation, NavPosePanTilt, NavPoseSolution
from nepi_interfaces.msg import Predict, PredictStatus, PredictProcess
from nepi_interfaces.msg import UpdateBool, UpdateFloat


from std_msgs.msg import UInt8, Int32, Float32, Empty, String, Bool, Header

from nepi_api.node_if import NodeClassIF
from nepi_api.messages_if import MsgIF
from nepi_api.connect_device_if_ptx import ConnectPTXDeviceIF
from nepi_api.connect_data_if_all import ConnectImagesAllIF


UPDATE_IMAGE_SUBS_RATE_HZ = 1
UPDATE_SAVE_DATA_CHECK_RATE_HZ = 10

TARGET_TOPIC_TIMEOUT_SEC = 2
TARGET_TRACK_TIMEOUT_SEC = 2
AUTO_SOURCE_RESET_TIMEOUT_SEC = 3.0

CAM_SWITCH_DELAY = 1.0

#########################################
# Node Class
#########################################

class NepiPanTiltAutoApp(object):

  PAN_MIN_MAX_DEG = 165
  TILT_MIN_MAX_DEG = 50

  SCAN_SWITCH_DEG = 5 # If angle withing this bound, switch dir
  SCAN_UPDATE_INTERVAL = .5

  MIN_SCAN_ANGLE = 30
  LIMIT_PADDING = 5

  # Reject a PT position feedback sample if it lands more than this far outside
  # the soft limits: the head cannot physically be past its hardstops, so such a
  # reading is a driver glitch (e.g. a one-cycle ~430 deg tilt spike). Wide enough
  # to never reject a legitimate near-hardstop reading (soft limits are padded
  # ~5 deg inside the hardstops), narrow enough to catch gross glitches.
  PT_POS_REJECT_MARGIN_DEG = 15

  AUTO_DEFAULT_SOURCE = 'Microstrain'
  MIN_AUTO_ANGLE = 30

  IMAGE_OPTIONS = ['IMAGES','DETECTIONS','TARGETS']
  IMAGE_NAMES = ['color_image','detections_image','targets_image']



  #####################
  
  node_if = None
  process_needs_update = False
  status_msg = PanTiltAutoAppStatus() 
  status_has_published = False
  status_update_rate = 1

  #####################

  available_pan_tilts = []
  selected_pan_tilt = "None"
  pt_connect_if = None

  pt_connected_topic = None
  pt_connected = False

  # Timestamped PT feedback subscriber + last stamp (own sub; the connect-IF
  # panTiltCb hook drops the stamp). Feeds the PT state buffer below.
  pt_pos_sub = None
  pt_last_timestamp = 0.0

  # State ring buffers (logic in nepi_auto_pt; instances created in __init__,
  # fed by async sensor callbacks). Used to reconstruct roll/pitch/pan/tilt at a
  # detection's image time for latency-correct target-vector estimation.
  imu_state_buffer = None
  pt_state_buffer = None

  pan_tilt_max_speed_dps = -999
  pan_tilt_avg_move_delay = 0.2
  pan_deg_per_sec = -999
  tilt_deg_per_sec = -999

  speed_ratio = 1.0
  pan_speed_ratio = 1.0
  tilt_speed_ratio = 1.0

  min_pan_softstop_deg = -PAN_MIN_MAX_DEG
  max_pan_softstop_deg = PAN_MIN_MAX_DEG
  min_tilt_softstop_deg = -TILT_MIN_MAX_DEG
  max_tilt_softstop_deg = TILT_MIN_MAX_DEG

  pt_stop = False,
  pan_home = False,
  tilt_home = False,
  pan_pos_update = None
  tilt_pos_update = None
  pan_ratio_update = None
  tilt_ratio_update = None
  pan_speed_ratio_update = None
  tilt_speed_ratio_update = None
  pan_click_position_udpate = None
  tilt_click_position_udpate = None

  goto_position = [0,0]
  # One-shot click->target-vector update for the auto controller. Set by
  # mouseClickCb when Stabilize/Sweep is active; read-and-cleared into
  # auto_data_dict['auto2_click_update'] in _autoStep (same one-shot
  # pattern as pt_stop / *_ratio_update to dodge the deepcopy/writeback race).
  click_update = None

  navpose_dict = None
  navpose_config = None
  navpose_update_rate = 1
  status_update_rate = 1

  current_position = None

  #####################
  targeting_topic = 'targets'
  targets_status_msg = None
  targets_status_msg_start = None
  targets_status_last_time = None
  targets_timeout = 3
  targets_last_time = 0
  targets_timestamp = 0
  targets_list = None


  #####################
  has_dual_mode = True
  has_zoom_mode = True
  has_night_mode = True

  has_scan_pan = True
  has_scan_tilt = True
  has_sin_pan = False
  has_sin_tilt = False
  has_homing = False
  has_set_home = False


  is_scanning = False
  pan_scanning = False
  tilt_scanning = False

  is_tracking = False
  pan_tracking = False
  tilt_tracking = False

  is_stabbing = False
  pan_stabbing = False
  tilt_stabbing = False


  
  is_night = False
  was_night = None
  night_mode_enabled = False

  zoom_mode_enabled = False

  detect_mode_enabled = False
  ###############
  # Tracking
  ###############

  tracking_topic = 'ai_track'

  track_enabled = False
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



  tracking_available_sources = []
  tracking_available_classes = []
  tracking_best_filter_options = copy.deepcopy(nepi_track.BEST_FILTER_OPTIONS)

  tracking_dict = copy.deepcopy(nepi_track.BLANK_SETTINGS_DICT)

  track_dict = None
  track_dict_check = None
  track_dict_timestamp = 0
  track_timeout = 3
  track_last_time = 0


  #####################
  # Scan
  scan_ready = False

  scan_enabled = False
  scan_pan_enabled = False
  scan_tilt_enabled = False

  #####################
  # Track

  targets_msg = None
  targets_last_time = 0
  
  track_ready = False

  track_enabled = False
  track_pan_enabled = False
  track_tilt_enabled = False


  track_range_min_m = 0
  track_range_max_m = 1000

  track_pan_error = 0
  track_tilt_error = 0

  track_if = None


  #####################
  # Stab

  navpose_msg = None
  last_navpose_time = 0

 
  available_stab_source_dict = dict()
  selected_stab_source = 'None'

  
  stab_source_connected_namespace = "None"
  stab_source_connected_message = None
  stab_source_connecting = False
  stab_source_connected = False
  last_stab_source_time = None

  stab_source_dict = None
  stab_source_lock = threading.Lock()
  stab_subpub_dict = None
  stab_subpub_lock = threading.Lock()

  stab_ready = False

  stab_enabled = False
  stab_pan_enabled = False
  stab_tilt_enabled = False


  #####################
  # Auto
  auto_night_mode_enabled = True

  available_auto_processes = list(nepi_auto_pt.PROCESSES_DICT.keys())
  auto_processes_dict = nepi_auto_pt.create_processes_dict()
  selected_auto_process = nepi_auto_pt.DEFAULT_PROCESS
  auto_process_ready = True

  auto_pan_min_deg = -PAN_MIN_MAX_DEG
  auto_pan_max_deg = PAN_MIN_MAX_DEG
  auto_tilt_min_deg = -TILT_MIN_MAX_DEG
  auto_tilt_max_deg = TILT_MIN_MAX_DEG

  auto_data_dict = nepi_auto_pt.get_blank_data_dict()
  auto_data_dict_last = None
  auto_dict_lock = threading.Lock()
  auto_pan_speed_start = 1.0
  auto_tilt_speed_start = 1.0
  auto_pan_adj = 0.0
  auto_tilt_adj = 0.0



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
 
  stream_quality = 1
  stream_rate = 20
  overlays_enabled = False

  selected_image_topics = ["None","None","None","None"]
  last_image_topics = ["None","None","None","None"]
  disable_image_pubs = [None,None,None,None]
  disable_image_save_pubs = [None,None,None,None]
  disables_need_update = True
  num_windows = 1
  full_screen_enabled = False
  image_stab_enabled = False

  available_image_topics = []  
  available_dimage_topics = []  
  available_timage_topics = []  

  images_all_if = None

  
  
  #######################
  ### Node Initialization
  DEFAULT_NODE_NAME = "app_pan_tilt_scan" # Can be overwitten by luanch command
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

    # Timestamped state ring buffers (logic in nepi_auto_pt; fed by the async
    # sensor callbacks in this node). Created before any subscriber so the feed
    # callbacks always have a buffer to append to.
    self.imu_state_buffer = nepi_auto_pt.TimeStateBuffer(['roll_deg', 'pitch_deg'])
    self.pt_state_buffer = nepi_auto_pt.TimeStateBuffer(['pan_deg', 'tilt_deg'])

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
        ###Pan Tilt
        #####################
        'selected_pan_tilt': {
            'namespace': self.node_namespace,
            'factory_val': self.selected_pan_tilt
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
        'scan_enabled': {
            'namespace': self.node_namespace,
            'factory_val': False
        },      
        'scan_pan_enabled': {
            'namespace': self.node_namespace,
            'factory_val': False
        },          
        'scan_tilt_enabled': {
            'namespace': self.node_namespace,
            'factory_val': False
        },     

        #####################
        ###Tracking
        #####################
        'tracking_dict': {
            'namespace': self.node_namespace,
            'factory_val': self.tracking_dict
        },
        # 'track_enabled': {
        #     'namespace': self.node_namespace,
        #     'factory_val': False
        # }, 
        # 'track_pan_enabled': {
        #     'namespace': self.node_namespace,
        #     'factory_val': False
        # },  
        # 'track_tilt_enabled': {
        #     'namespace': self.node_namespace,
        #     'factory_val': False
        # },   


        #####################
        ###Stab
        #####################

        'selected_stab_source': {
            'namespace': self.node_namespace,
            'factory_val': self.selected_stab_source
        },
        'stab_enabled': {
            'namespace': self.node_namespace,
            'factory_val': False
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
        ### Auto
        #####################
         'auto_night_mode_enabled': {
            'namespace': self.node_namespace,
            'factory_val': self.auto_night_mode_enabled
        },
         'auto_processes_dict': {
            'namespace': self.node_namespace,
            'factory_val': self.auto_processes_dict
        },
        'selected_auto_process': {
            'namespace': self.node_namespace,
            'factory_val': self.selected_auto_process
        },
        'auto_pan_min_deg': {
            'namespace': self.node_namespace,
            'factory_val': self.auto_pan_min_deg
        },
        'auto_pan_max_deg': {
            'namespace': self.node_namespace,
            'factory_val': self.auto_pan_max_deg
        },
        'auto_tilt_min_deg': {
            'namespace': self.node_namespace,
            'factory_val': self.auto_tilt_min_deg
        },
        'auto_tilt_max_deg': {
            'namespace': self.node_namespace,
            'factory_val': self.auto_tilt_max_deg
        },


        #####################
        ###Image Viewer
        #####################
        # 'full_screen_enabled': {
        #     'namespace': self.node_namespace,
        #     'factory_val': self.full_screen_enabled
        # },
        'stream_quality': {
            'namespace': self.node_namespace,
            'factory_val': self.stream_quality
        },
        'stream_rate': {
            'namespace': self.node_namespace,
            'factory_val': self.stream_rate
        },
        'overlays_enabled': {
            'namespace': self.node_namespace,
            'factory_val': self.overlays_enabled
        },
        'selected_image_topics': {
            'namespace': self.node_namespace,
            'factory_val': self.selected_image_topics
        },
        'num_windows': {
            'namespace': self.node_namespace,
            'factory_val': self.num_windows
        },
        'has_dual_mode': {
            'namespace': self.node_namespace,
            'factory_val': self.has_dual_mode
          },
        'has_zoom_mode': {
            'namespace': self.node_namespace,
            'factory_val': self.has_zoom_mode
          },
        'has_night_mode': {
            'namespace': self.node_namespace,
            'factory_val': self.has_night_mode
        }

    }

    # Publishers Config Dict ####################
    self.PUBS_DICT = {
        'status_pub': {
            'namespace': self.node_namespace,
            'topic': 'status',
            'msg': PanTiltAutoAppStatus,
            'qsize': 1,
            'latch': True
        },
        #####################
        ### Tracking
        #####################
        # 'ai_track': {
        #     'msg': Target,
        #     'namespace': self.node_namespace,
        #     'topic': self.tracking_topic,
        #     'qsize': 1,
        #     'latch': True
        # },
        # 'track_status': {
        #     'msg': TrackingStatus,
        #     'namespace': self.node_namespace + '/' + self.tracking_topic,
        #     'topic': 'status',
        #     'qsize': 1,
        #     'latch': True
        # }
    }

    # Subscribers Config Dict ####################
    self.SUBS_DICT = {

        #####################
        ### Pan Tilt
        #####################
        'select_pan_and_tilt': {
            'namespace': self.node_namespace,
            'topic': 'select_pt_device',
            'msg': String,
            'qsize': None,
            'callback': self.selectTopicCb, 
            'callback_args': ()
        },
        'set_pan_pos_deg': {
            'namespace': self.node_namespace,
            'topic': 'set_pan_pos_deg',
            'msg': Float32,
            'qsize': 1,
            'callback': self.setPanPosDegCb,
            'callback_args': ()
        },
        'set_tilt_pos_deg': {
            'namespace': self.node_namespace,
            'topic': 'set_tilt_pos_deg',
            'msg': Float32,
            'qsize': 1,
            'callback': self.setTiltPosDegCb,
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
        'pan_tilt_home': {
            'namespace': self.node_namespace,
            'topic': 'pan_tilt_home',
            'msg': Empty,
            'qsize': 1,
            'callback': self.panTiltHomeCb,
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
        'pan_tilt_stop': {
            'namespace': self.node_namespace,
            'topic': 'pan_tilt_stop',
            'msg': Empty,
            'qsize': 1,
            'callback': self.ptStopCb,
            'callback_args': ()
        },

        #####################
        ###Scan
        #####################
        'set_scan_enable': {
            'namespace': self.node_namespace,
            'topic': 'set_scan_enable',
            'msg': Bool,
            'qsize': 1,
            'callback': self.setScanCb,
            'callback_args': ()
        },
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

        #####################
        ###Tracking
        #####################

        'set_tracking_targets_topic': {
            'namespace': self.node_namespace + '/' + self.tracking_topic,
            'topic': 'set_targets_topic',
            'msg': String,
            'qsize': 10,
            'callback': self.setTrackTargetsTopicCb, 
            'callback_args': ()
        },
        # 'set_tracking_source_topic': {
        #     'namespace': self.node_namespace + '/' + self.tracking_topic,
        #     'topic': 'set_source_topic',
        #     'msg': String,
        #     'qsize': 10,
        #     'callback': self.setTrackSourceTopicCb, 
        #     'callback_args': ()
        # },
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
        'set_track_enable': {
            'namespace': self.node_namespace,
            'topic': 'set_track_enable',
            'msg': Bool,
            'qsize': 1,
            'callback': self.setTrackCb, 
            'callback_args': ()
        },
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
      

        #####################
        ### Stab
        #####################

         'set_stab_source': {
            'namespace': self.node_namespace,
            'topic': 'set_stab_source',
            'msg': String,
            'qsize': 10,
            'callback': self.setStabSourceCb, 
            'callback_args': ()
        },       
        'set_stab_enable': {
            'namespace': self.node_namespace,
            'topic': 'set_stab_enable',
            'msg': Bool,
            'qsize': 1,
            'callback': self.setStabCb,
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
        ### Auto
        #####################

        'reload_autos': {
            'namespace': self.node_namespace,
            'topic': 'reload_auto_processes',
            'msg': Empty,
            'qsize': 10,
            'callback': self.reloadAutosCb, 
            'callback_args': ()
        },

        'set_auto_process': {
            'namespace': self.node_namespace,
            'topic': 'set_auto_process',
            'msg': String,
            'qsize': 10,
            'callback': self.setAutoProcessCb, 
            'callback_args': ()
        },

        'set_auto_update_rate': {
            'namespace': self.node_namespace,
            'topic': 'set_auto_update_rate',
            'msg': Float32,
            'qsize': 1,
            'callback': self.setAutoUpdateRateCb,
            'callback_args': ()
        },
        'set_auto_control_value': {
            'namespace': self.node_namespace,
            'topic': 'set_auto_control_value',
            'msg': UpdateFloat,
            'qsize': 1,
            'callback': self.setAutoControlCb,
            'callback_args': ()
        },
        'set_auto_night_enable': {
            'namespace': self.node_namespace,
            'topic': 'set_auto_night_enable',
            'msg': Bool,
            'qsize': None,
            'callback': self.setAutoNightEnableCb, 
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
        'has_dual_mode': {
            'namespace': self.node_namespace,
            'topic': 'set_has_dual_mode',
            'msg': Bool,
            'qsize': None,
            'callback': self.setHasDualModeCb, 
            'callback_args': ()
        },
        'has_zoom_mode': {
            'namespace': self.node_namespace,
            'topic': 'set_has_zoom_mode',
            'msg': Bool,
            'qsize': None,
            'callback': self.setHasZoomModeCb, 
            'callback_args': ()
        },
        'has_night_mode': {
            'namespace': self.node_namespace,
            'topic': 'set_has_night_mode',
            'msg': Bool,
            'qsize': None,
            'callback': self.setHasNightModeCb, 
            'callback_args': ()
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
        'set_full_screen_enable': {
            'namespace': self.node_namespace,
            'topic': 'set_full_screen_enable',
            'msg': Bool,
            'qsize': 1,
            'callback': self.setFullScreenEnableCb,
            'callback_args': ()
        },
        'set_image_stream_rate': {
            'namespace': self.node_namespace,
            'topic': 'set_image_stream_rate',
            'msg': Float32,
            'qsize': 1,
            'callback': self.setImageStreamRateCb,
            'callback_args': ()
        },
        'set_image_overlays_enable': {
            'namespace': self.node_namespace,
            'topic': 'set_image_overlays_enable',
            'msg': Bool,
            'qsize': 1,
            'callback': self.setImageOveralysEnableCb,
            'callback_args': ()
        },
        'set_image_dual_enable': {
            'namespace': self.node_namespace,
            'topic': 'set_image_dual_enable',
            'msg': Bool,
            'qsize': None,
            'callback': self.setIsDualCb, 
            'callback_args': ()
        },
        'set_image_night_enable': {
            'namespace': self.node_namespace,
            'topic': 'set_image_night_enable',
            'msg': Bool,
            'qsize': None,
            'callback': self.setIsNightCb, 
            'callback_args': ()
        },
        'set_image_zoom_enable': {
            'namespace': self.node_namespace,
            'topic': 'set_image_zoom_enable',
            'msg': Bool,
            'qsize': 1,
            'callback': self.setImageZoomModeCb,
            'callback_args': ()
        },
        'set_image_detect_enable': {
            'namespace': self.node_namespace,
            'topic': 'set_image_detect_enable',
            'msg': Bool,
            'qsize': 1,
            'callback': self.setImageDetectModeCb,
            'callback_args': ()
        },
        'set_image_stab_enable': {
            'namespace': self.node_namespace,
            'topic': 'set_image_stab_enable',
            'msg': Bool,
            'qsize': 1,
            'callback': self.setImageStabModeCb,
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
    
    self.images_all_if = ConnectImagesAllIF(
                msg_if = self.msg_if,
                node_if = self.node_if
    )

    nepi_sdk.wait()

    ##############################
    self.initCb(do_updates = True)

    ##############################
    # Start updater process
    nepi_sdk.start_timer_process(1.0, self.updaterCb, oneshot = True)
    nepi_sdk.start_timer_process(0.2, self.updaterPanTiltCb, oneshot = True)

    self.msg_if.pub_warn("Starting status pub")
    nepi_sdk.start_timer_process(0.5, self.publishStatusCb)

    # Scan/track/stab motion is handled by the unified auto process loop
    # (updaterAutoSolutionCb, started below). The legacy per-axis scan/track
    # timer processes were removed in the Auto refactor.

    nepi_sdk.start_timer_process(1.0, self.updaterTrackingCb, oneshot = True)
    nepi_sdk.start_timer_process(1.0, self.updaterTrackingStateCb, oneshot = True)

    nepi_sdk.start_timer_process(1.0, self.updaterStabCb, oneshot = True)

    # Start the dedicated auto control loop. Replaces the self-rescheduling
    # oneshot timer with a single fixed-rate thread paced on a monotonic clock
    # (NTP-immune) with a whole-step crash guard.
    self._auto_loop_stop = False
    self._auto_data_dict_prev = None
    self._auto_last_cycle_ms = 0.0
    self._auto_last_overrun = 0
    self.msg_if.pub_info(" Starting Auto Loop Thread")
    self._auto_thread = threading.Thread(target=self._autoLoop, name="pt_auto_loop", daemon=True)
    self._auto_thread.start()




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
        ###Pan Tilt
        #####################
        
        self.selected_pan_tilt = self.node_if.get_param('selected_pan_tilt')
        self.speed_ratio = self.node_if.get_param('speed_ratio')
        self.pan_speed_ratio = self.node_if.get_param('pan_speed_ratio')
        self.tilt_speed_ratio = self.node_if.get_param('tilt_speed_ratio')


        #####################
        ###Imaging
        #####################
        self.num_windows = self.node_if.get_param('num_windows')
        self.selected_image_topics = self.node_if.get_param('selected_image_topics')
        self.msg_if.pub_warn("Starting with selected images: " + str(self.selected_image_topics))
        #self.full_screen_enabled = self.node_if.get_param('full_screen_enabled')
        self.stream_quality = self.node_if.get_param('stream_quality')
        self.stream_rate = self.node_if.get_param('stream_rate')
        self.overlays_enabled = self.node_if.get_param('overlays_enabled')
           
        self.has_dual_mode = self.node_if.get_param('has_dual_mode')
        self.has_night_mode = self.node_if.get_param('has_night_mode')
        self.has_zoom_mode = self.node_if.get_param('has_zoom_mode')

        #####################
        ###Scan
        #####################

        # Per-axis scan is deprecated; only the whole-unit Sweep enable is used.
        # Always boot in Manual mode: force Sweep OFF at startup, ignoring the saved value.
        self.setScan(False)


        #####################
        ###Track
        #####################
        # Per-axis track is deprecated; only the whole-unit AI Tracking enable is used.
        # Always boot in Manual mode: force AI Tracking OFF at startup, ignoring the saved value.
        self.setTrack(False)


        #####################
        ###Stab
        #####################

        self.selected_stab_source = self.node_if.get_param('selected_stab_source')
      
        # Per-axis stab is deprecated; only the whole-unit Stabilize enable is used.
        # Always boot in Manual mode: force Stabilize OFF at startup, ignoring the saved value.
        self.setStab(False)

      


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
        tracking_dict['source_topic'] = 'None'
        self.tracking_dict = tracking_dict
        self.tracking_targets_topic = tracking_dict['targets_topic']
        self.source_targets_topic = tracking_dict['source_topic']



        #####################
        ### Auto
        #####################
        self.auto_pan_min_deg = self.node_if.get_param('auto_pan_min_deg')
        self.auto_pan_max_deg = self.node_if.get_param('auto_pan_max_deg')
        self.auto_tilt_min_deg = self.node_if.get_param('auto_tilt_min_deg')
        self.auto_tilt_max_deg = self.node_if.get_param('auto_tilt_max_deg')

        self.auto_pan_speed_start = self.pan_speed_ratio
        self.auto_tilt_speed_start = self.tilt_speed_ratio
        self.auto_night_mode_enabled = self.node_if.get_param('auto_night_mode_enabled')
        auto_processes_dict =  self.node_if.get_param('auto_processes_dict')
        auto_processes_dict = nepi_auto_pt.update_processes_dict(auto_processes_dict)
        self.auto_processes_dict = auto_processes_dict

        # Always boot with telemetry disabled for any auto process that exposes
        # this control, ignoring saved values from prior runs.
        for auto_process in self.auto_processes_dict.keys():
            auto_controls_dict = self.auto_processes_dict[auto_process].get('auto_controls_dict', {})
            if 'telem_enabled' in auto_controls_dict.keys():
                auto_controls_dict['telem_enabled'] = 0
                self.auto_processes_dict[auto_process]['auto_controls_dict'] = auto_controls_dict
        self.node_if.set_param('auto_processes_dict', self.auto_processes_dict)

        selected_auto_process = self.node_if.get_param('selected_auto_process')
        if selected_auto_process in auto_processes_dict.keys():
            self.selected_auto_process = selected_auto_process
        else:
            self.selected_auto_process = list(auto_processes_dict.keys())[0]
        self.auto_process_ready = True





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

    selected_pan_tilt = copy.deepcopy(self.selected_pan_tilt)
    last_available = copy.deepcopy(self.available_pan_tilts)

    topics = nepi_sdk.find_topics_by_msg('DevicePTXStatus', topics_list = self.active_topics, types_list = self.active_topic_types)
    available_pan_tilts = []
    for topic in topics:
      available_pan_tilts.append(topic.replace('/status',''))
    if available_pan_tilts != last_available:
        self.msg_if.pub_warn("Available Pan Tilts Updated: " + str(available_pan_tilts))
    self.available_pan_tilts = available_pan_tilts


    ####################
    if self.pt_connected_topic is not None:
      if self.pt_connected_topic not in self.available_pan_tilts:
        success = self.unsubscribe_pt_topic()
    if selected_pan_tilt == 'None' and len(self.available_pan_tilts) > 0:
        self.selected_pan_tilt = self.available_pan_tilts[0]


    was_connected = copy.deepcopy(self.pt_connected)
    if self.selected_pan_tilt in self.available_pan_tilts and self.pt_connected_topic != selected_pan_tilt:
        success = self.subscribe_pt_topic(self.selected_pan_tilt)
    elif self.pt_connect_if is not None:
        self.pt_connected = self.pt_connect_if.check_connection()
        if self.pt_connected == True:
            limits = self.pt_connect_if.get_pan_tilt_soft_limits()
            #self.msg_if.pub_warn("setting scan limits: " + str(limits))
            if limits is not None:
                self.min_pan_softstop_deg = round(limits[0], 0) + self.LIMIT_PADDING
                self.max_pan_softstop_deg = round(limits[1], 0) - self.LIMIT_PADDING
                self.min_tilt_softstop_deg = round(limits[2], 0) + self.LIMIT_PADDING
                self.max_tilt_softstop_deg = round(limits[3], 0) - self.LIMIT_PADDING
                # Re-clamp the auto travel window to the device's reported soft limits
                self.auto_pan_min_deg = max(self.auto_pan_min_deg, self.min_pan_softstop_deg)
                self.auto_pan_max_deg = min(self.auto_pan_max_deg, self.max_pan_softstop_deg)
                self.auto_tilt_min_deg = max(self.auto_tilt_min_deg, self.min_tilt_softstop_deg)
                self.auto_tilt_max_deg = min(self.auto_tilt_max_deg, self.max_tilt_softstop_deg)
    else:
        self.pt_connected = False

    ######################
    topics = nepi_sdk.find_topics_by_msg('Image', topics_list = self.active_topics, types_list = self.active_topic_types)

    available_image_topics = []
    available_dimage_topics = []
    available_timage_topics = []
    for topic in topics:
      if 'color_image' in topic:
          available_image_topics.append(topic)   
      if 'detections_image' in topic:
          available_dimage_topics.append(topic) 
      if 'track_image' in topic:
          available_timage_topics.append(topic)
     
    last_itopics = copy.deepcopy(self.available_image_topics)
    if available_image_topics != last_itopics:
        self.msg_if.pub_warn("Updating image topics: " + str(available_image_topics))
    self.available_image_topics = available_image_topics

    last_dtopics = copy.deepcopy(self.available_dimage_topics)
    if available_dimage_topics != last_dtopics:
        self.msg_if.pub_warn("Updating detect topics: " + str(available_dimage_topics))
    self.available_dimage_topics = available_dimage_topics

    last_ttopics = copy.deepcopy(self.available_timage_topics)
    if available_timage_topics != last_ttopics:
        self.msg_if.pub_warn("Updating track topics: " + str(available_timage_topics))
    self.available_timage_topics = available_timage_topics


    ###################################
    # Update Status Images

    num_windows = copy.deepcopy(self.num_windows)
    daul_mode = (num_windows > 1)
    detect_mode = self.detect_mode_enabled
    night_mode = self.night_mode_enabled
    zoom_mode = self.zoom_mode_enabled
    
    avail_itopics = copy.deepcopy(self.available_image_topics)
    self.status_msg.avail_image_topics = avail_itopics
    avail_dtopics = copy.deepcopy(self.available_dimage_topics)


    selected_image_topics = copy.deepcopy(self.selected_image_topics)
    for i, topic in enumerate(selected_image_topics):
       if topic != 'None':
          if topic not in avail_itopics:
             selected_image_topics[i] = 'None'
    self.status_msg.selected_image_topics = selected_image_topics

    if selected_image_topics != self.last_image_topics:
        self.disables_need_update = True
    if True: #selected_image_topics != self.last_image_topics:
        for i, topic in enumerate(selected_image_topics):
            if topic == 'None':
                self.disable_image_pubs[i] = None
                self.disable_image_save_pubs[i] = None
            else:
                idtopic = topic.split('/idx')[0] + '/idx/disable'
                self.disable_image_pubs[i] = nepi_sdk.create_publisher(idtopic, Bool)
                isdtopic = topic.split('/idx')[0] + '/save_data/disable'
                self.disable_image_save_pubs[i] = nepi_sdk.create_publisher(isdtopic, Bool)

    if self.disables_need_update == True:
        self.disables_need_update = False
        if night_mode == False:
            if self.disable_image_pubs[2] is not None:
                nepi_sdk.publish_pub(self.disable_image_pubs[2],True)
            if self.disable_image_pubs[3] is not None:
                nepi_sdk.publish_pub(self.disable_image_pubs[3],True)
            nepi_sdk.sleep(CAM_SWITCH_DELAY)
            if self.disable_image_pubs[0] is not None:
                nepi_sdk.publish_pub(self.disable_image_pubs[0],False)
            if self.disable_image_pubs[1] is not None:
                nepi_sdk.publish_pub(self.disable_image_pubs[1],False)
        else:
            if self.disable_image_pubs[0] is not None:
                nepi_sdk.publish_pub(self.disable_image_pubs[0],True)
            if self.disable_image_pubs[1] is not None:
                nepi_sdk.publish_pub(self.disable_image_pubs[1],True)
            nepi_sdk.sleep(CAM_SWITCH_DELAY)
            if self.disable_image_pubs[2] is not None:
                nepi_sdk.publish_pub(self.disable_image_pubs[2],False)
            if self.disable_image_pubs[3] is not None:
                nepi_sdk.publish_pub(self.disable_image_pubs[3],False)

        if night_mode == False:
            if self.disable_image_save_pubs[2] is not None:
                nepi_sdk.publish_pub(self.disable_image_save_pubs[2],True)
            if self.disable_image_save_pubs[3] is not None:
                nepi_sdk.publish_pub(self.disable_image_save_pubs[3],True)
            nepi_sdk.sleep(CAM_SWITCH_DELAY)
            if self.disable_image_save_pubs[0] is not None:
                nepi_sdk.publish_pub(self.disable_image_save_pubs[0],False)
            if self.disable_image_save_pubs[1] is not None:
                nepi_sdk.publish_pub(self.disable_image_save_pubs[1],False)
        else:
            if self.disable_image_save_pubs[0] is not None:
                nepi_sdk.publish_pub(self.disable_image_save_pubs[0],True)
            if self.disable_image_save_pubs[1] is not None:
                nepi_sdk.publish_pub(self.disable_image_save_pubs[1],True)
            nepi_sdk.sleep(CAM_SWITCH_DELAY)
            if self.disable_image_save_pubs[2] is not None:
                nepi_sdk.publish_pub(self.disable_image_save_pubs[2],False)
            if self.disable_image_save_pubs[3] is not None:
                nepi_sdk.publish_pub(self.disable_image_save_pubs[3],False)
    self.last_image_topics = selected_image_topics
    


    if night_mode == False:
        if zoom_mode == True:
            single_image_index = 1
        else:
            single_image_index = 0
    else:
        if zoom_mode == True:
            single_image_index = 3
        else:
            single_image_index = 2
    self.status_msg.single_image_index = single_image_index

    display_image_topics = copy.deepcopy(selected_image_topics)
    # self.msg_if.pub_warn("################:")
    # self.msg_if.pub_warn("Start Status detect images: " + str(display_image_topics))
    # self.msg_if.pub_warn("Got detect sources: " + str(avail_dtopics))
    # self.msg_if.pub_warn("Got detect enabled: " + str(detect_mode))
    has_detect_mode = False
    for i, topic in enumerate(selected_image_topics):
        if topic != 'None' and ((i == 0 and night_mode == False) or (i == 2 and night_mode == True)):
            self.tracking_dict['source_topic'] = topic
            dtopic = topic.replace('color_image','detections_image')
            has_detect_mode = True
            if detect_mode == True:
                #self.msg_if.pub_warn("Looking for dtopic: " + str(dtopic) + " in " + str(avail_dtopics))
                if dtopic in avail_dtopics:
                    display_image_topics[i] = dtopic

    #self.msg_if.pub_warn("Pub Status detect images: " + str(display_image_topics))


    self.status_msg.has_detect_mode = has_detect_mode
    self.status_msg.display_image_topics = display_image_topics


    #######################################
    ## Update Control Values

    auto_settings_dict = copy.deepcopy(self.auto_processes_dict[self.selected_auto_process])
    self.status_msg.auto_update_rate = auto_settings_dict['auto_update_rate']

    auto_control_names = []
    auto_control_values = []
    auto_controls_dict = auto_settings_dict['auto_controls_dict']
    for control in auto_controls_dict.keys():
        auto_control_names.append(control)
        auto_control_values.append(auto_controls_dict[control])
    self.status_msg.auto_control_names = auto_control_names
    self.status_msg.auto_control_values = auto_control_values



    nepi_sdk.start_timer_process(1.0, self.updaterCb, oneshot = True)


  def updaterPanTiltCb(self,timer):
    #self.msg_if.pub_warn("Updater Called")
    if self.pt_connect_if is not None:
      self.current_position = self.pt_connect_if.get_pan_tilt_position()
      #self.msg_if.pub_warn("current_position: " + str(current_position))



     


  ##############################
  ## Node PT Commands

  def is_manual_mode(self):
      # Manual mode = none of the auto modes (Stabilize / Sweep / AI Tracking)
      # are enabled. Home commands are only honored in manual mode.
      return not (self.stab_enabled or self.scan_enabled or self.track_enabled)

  def _home_set_max_speed(self):
      # Force BOTH axes to max speed before a home move, then pause so the speed
      # command actually reaches the PTX controller BEFORE the position move is
      # issued.
      #
      # Why the pause matters: the driver's absolute-position move (serial 'MML')
      # carries no speed of its own -- it runs at whatever axis speed is latched
      # when it arrives. set_speed_ratio() and the goto are two separate async ROS
      # topics handled on separate driver threads, so without ordering the move can
      # win the race and latch the OLD speed. After a jog that left the axis near
      # zero speed, that makes Home crawl to the target (observed: "new speed 1.00 /
      # cur speed 0.00" immediately before a slow move). The whole-unit setter is
      # used (not the per-axis ones) because the device IF de-dups per-axis speed
      # commands and can drop them; the combined setter is never gated.
      if self.pt_connect_if is None:
          return
      self.pt_connect_if.set_speed_ratio(1.0)
      nepi_sdk.sleep(0.3)

  def panTiltHomeCb(self, msg):
      # Only act in manual mode; drive both axes home at max speed.
      if not self.is_manual_mode():
          self.msg_if.pub_info("Home ignored: only available in manual mode")
          return
      if self.pt_connect_if is not None:
          self._home_set_max_speed()
          self.pt_connect_if.go_home()

  def panHomeCb(self, msg):
    # Pan Home only acts in manual mode. Drive the pan axis to home (0 deg) at
    # max speed straight through the driver, matching how STOP and the manual
    # sliders bypass the auto controller.
    if not self.is_manual_mode():
        self.msg_if.pub_info("Pan Home ignored: only available in manual mode")
        return
    if self.pt_connect_if is not None:
        self._home_set_max_speed()
        self.pt_connect_if.goto_to_pan_position(0.0)

  def tiltHomeCb(self, msg):
    # Tilt Home only acts in manual mode. Drive the tilt axis to home (0 deg) at
    # max speed straight through the driver.
    if not self.is_manual_mode():
        self.msg_if.pub_info("Tilt Home ignored: only available in manual mode")
        return
    if self.pt_connect_if is not None:
        self._home_set_max_speed()
        self.pt_connect_if.goto_to_tilt_position(0.0)

  def ptStopCb(self,msg):
    self.pt_stop = True

    

  def getPanClickEnabled(self):
     return (self.click_pan_enabled == True) and (self.scan_pan_enabled == False and self.track_pan_enabled == False and self.stab_pan_enabled == False)

  def getTiltClickEnabled(self):
     return (self.click_tilt_enabled == True) and (self.scan_tilt_enabled == False and self.track_tilt_enabled == False and self.stab_tilt_enabled == False)


  def setPanPosDegCb(self, msg):
      self.setPanPosDeg(msg.data)

  def setPanPosDeg(self, pos_deg):
        # Absolute pan-position command from the RUI position entry. Honored only in
        # manual mode (in an auto mode the controller owns the axis). Mirrors the
        # manual-click goto path the user confirmed works -- a direct driver goto.
        if self.pt_connect_if is not None and self.is_manual_mode():
            self.goto_position[0] = pos_deg
            self.pt_connect_if.goto_to_pan_position(pos_deg)

  def setTiltPosDegCb(self, msg):
      self.setTiltPosDeg(msg.data)

  def setTiltPosDeg(self, pos_deg):
        if self.pt_connect_if is not None and self.is_manual_mode():
            self.goto_position[1] = pos_deg
            self.pt_connect_if.goto_to_tilt_position(pos_deg)


  def setSpeedRatioCb(self, msg):
      self.setSpeedRatio(msg.data)

  def setSpeedRatio(self, speed_ratio):
        speed_ratio = nepi_utils.check_ratio(speed_ratio)
        if speed_ratio < 0.1:
            speed_ratio = 0.1
        self.speed_ratio = speed_ratio
        self.pan_speed_ratio = speed_ratio
        self.tilt_speed_ratio = speed_ratio
        if self.pt_connect_if is not None:
            self.pt_connect_if.set_speed_ratio(speed_ratio)
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
        if self.pt_connect_if is not None:
            self.pt_connect_if.set_pan_speed_ratio(speed_ratio)
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
        if self.pt_connect_if is not None:
            self.pt_connect_if.set_tilt_speed_ratio(speed_ratio)
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('tilt_speed_ratio', self.tilt_speed_ratio)

 
  ##########################################
  # SCAN

  def setScanCb(self, msg):
        enabled = msg.data
        self.msg_if.pub_info("Setting scan pan: " + str(enabled))
        self.setScan(enabled)


  def setScan(self,enabled):
        was_scanning = copy.deepcopy(self.scan_pan_enabled)
        self.scan_enabled = enabled
        if enabled and self.stab_enabled:
            # Stabilize and Sweep are mutually exclusive.
            self.stab_enabled = False
            if self.node_if is not None:
                self.node_if.set_param('stab_enabled', self.stab_enabled)
        self.publish_status()
        self.node_if.set_param('scan_enabled', enabled)
        

  def setScanPanCb(self, msg):
        enabled = msg.data
        self.msg_if.pub_info("Setting scan pan: " + str(enabled))
        self.setScanPan(enabled)


  def setScanPan(self,enabled):
        was_scanning = copy.deepcopy(self.scan_pan_enabled)
        self.scan_pan_enabled = enabled
        self.publish_status()
        self.node_if.set_param('scan_pan_enabled', enabled)
        


  def setScanTiltCb(self, msg):
        enabled = msg.data
        self.msg_if.pub_info("Setting scan tilt: " + str(enabled))
        self.setScanTilt(enabled)



  def setScanTilt(self,enabled):
        was_scanning = copy.deepcopy(self.scan_tilt_enabled)
        self.scan_tilt_enabled = enabled
        self.publish_status()  
        self.node_if.set_param('scan_tilt_enabled', self.scan_tilt_enabled)



  ##########################################
  # TRACK

  def setTrackCb(self, msg):
        enabled = msg.data
        self.msg_if.pub_info("Setting track pan: " + str(enabled))
        self.setTrack(enabled)


  def setTrack(self,enabled):      
        self.track_enabled = enabled
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('track_enabled', self.track_enabled)

  def setTrackPanCb(self, msg):
        enabled = msg.data
        self.msg_if.pub_info("Setting track pan: " + str(enabled))
        self.setTrackPan(enabled)


  def setTrackPan(self,enabled):      
        self.track_pan_enabled = enabled
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('track_pan_enabled', self.track_pan_enabled)

  def setTrackTiltCb(self, msg):
        enabled = msg.data
        self.msg_if.pub_info("Setting track tilt: " + str(enabled))
        self.setTrackTilt(enabled)

  def setTrackTilt(self,enabled):
        self.track_tilt_enabled = enabled
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('track_tilt_enabled', self.track_tilt_enabled)

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
        self.msg_if.pub_info("Setting track class filter to: " + str(value))
        #self.tracking_dict['class_filters'] = value
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
    # Auto
    #############################

  def setAutoPanWindowCb(self, msg):
      adj_min_deg = msg.start_range
      adj_max_deg = msg.stop_range
      if adj_min_deg > adj_max_deg:
        self.msg_if.pub_info("invalid range: " + "%.2f" % adj_min_deg  + " " + "%.2f" % adj_max_deg )
      else:
        self.msg_if.pub_info("Setting auto pan limits to: " + "%.2f" % adj_min_deg  + " " + "%.2f" % adj_max_deg )
        self.setAutoPanWindow(adj_min_deg,adj_max_deg)

  def setAutoPanWindow(self, min_deg, max_deg):
        if max_deg > min_deg and abs(max_deg - min_deg) >= self.MIN_AUTO_ANGLE:
            if max_deg > self.max_pan_softstop_deg:
                max_deg = self.max_pan_softstop_deg
            if min_deg < self.min_pan_softstop_deg:
                min_deg = self.min_pan_softstop_deg
            self.auto_pan_min_deg = min_deg
            self.auto_pan_max_deg = max_deg
            #self.msg_if.pub_info("Auto Pan limits set to: " + "%.2f" % min_deg  + " " + "%.2f" % max_deg )
            self.publish_status()
            self.node_if.set_param('auto_pan_min_deg', min_deg)
            self.node_if.set_param('auto_pan_max_deg', max_deg)
            


  def setAutoTiltWindowCb(self, msg):
      adj_min_deg = msg.start_range
      adj_max_deg = msg.stop_range
      self.msg_if.pub_info("Setting auto tilt limits to: " + "%.2f" % adj_min_deg  + " " + "%.2f" % adj_max_deg )
      self.setAutoTiltWindow(adj_min_deg,adj_max_deg)


  def setAutoTiltWindow(self, min_deg, max_deg):
      if max_deg > min_deg and abs(max_deg - min_deg) >= self.MIN_AUTO_ANGLE:
          if max_deg > self.max_tilt_softstop_deg:
              max_deg = self.max_tilt_softstop_deg
          if min_deg < self.min_tilt_softstop_deg:
              min_deg = self.min_tilt_softstop_deg
          self.auto_tilt_min_deg = min_deg
          self.auto_tilt_max_deg = max_deg
          #self.msg_if.pub_info("Auto Tilt limits set to: " + "%.2f" % min_deg  + " " + "%.2f" % max_deg )
          self.publish_status()
          self.node_if.set_param('auto_tilt_min_deg', min_deg)
          self.node_if.set_param('auto_tilt_max_deg', max_deg)

  def setAutoNightEnableCb(self, msg):
      enabled = msg.data
      self.msg_if.pub_info("Setting Is Night Enabled: " + str(enabled))
      self.setAutoNightEnable(enabled)

  def setAutoNightEnable(self,enabled):
    self.auto_night_mode_enabled = enabled
    self.publish_status()
    if self.node_if is not None:
        self.node_if.set_param('auto_night_mode_enabled', enabled)



  def setIsDualCb(self, msg):
    enabled = msg.data
    if enabled == True:
        self.setNumWindows(2)
    else:
        self.setNumWindows(1)
    self.publish_status()

  def setIsNightCb(self, msg):
    enabled = msg.data
    self.night_mode_enabled = enabled
    self.disables_need_update = True
    self.publish_status()


  def setImageZoomModeCb(self, msg):
    enabled = msg.data
    self.zoom_mode_enabled = enabled
    self.publish_status()

  def setImageDetectModeCb(self,msg):
    enabled = msg.data
    self.detect_mode_enabled = enabled
    self.publish_status()

  def setImageStabModeCb(self,msg):
    enabled = msg.data
    self.image_stab_enabled = enabled
    self.publish_status()

  def setPanClickCb(self, msg):
      enabled = msg.data
      self.msg_if.pub_info("Setting Click Pan Enabled: " + str(enabled))
      self.setPanClick(enabled)

  def setPanClick(self,enabled):
    self.click_pan_enabled = enabled
    self.publish_status()


  def setTiltClickCb(self, msg):
      enabled = msg.data
      self.msg_if.pub_info("Setting Click Tilt: " + str(enabled))
      self.setTiltClick(enabled)

  def setTiltClick(self,enabled):
    self.click_tilt_enabled = enabled
    self.publish_status()

  def setHasDualModeCb(self, msg):
      enabled = msg.data
      self.msg_if.pub_info("Setting Has Dual Mode: " + str(enabled))
      self.has_dual_mode = enabled
      if self.node_if is not None:
            self.node_if.set_param('has_dual_mode', enabled)
            #self.node_if.save_config()

  def setHasNightModeCb(self, msg):
      enabled = msg.data
      self.msg_if.pub_info("Setting  Has Night Mode: " + str(enabled))
      self.has_night_mode = enabled
      if self.node_if is not None:
            self.node_if.set_param('has_night_mode', enabled)
            #self.node_if.save_config()

  def setHasZoomModeCb(self, msg):
      enabled = msg.data
      self.msg_if.pub_info("Setting  Has Zoom Mode: " + str(enabled))
      self.has_zoom_mode = enabled
      if self.node_if is not None:
            self.node_if.set_param('has_zoom_mode', enabled)
            #self.node_if.save_config()


  def selectTopicCb(self,msg):
    selected_pan_tilt = msg.data
    if selected_pan_tilt in self.available_pan_tilts:
      self.selected_pan_tilt = selected_pan_tilt
      self.publish_status()
      if self.node_if is not None:
        self.msg_if.pub_warn("selected_pan_tilt: " + str(selected_pan_tilt))
        self.node_if.set_param('selected_pan_tilt', selected_pan_tilt)
    

  def getPositionWithinSoftLimits(self, pan_deg, tilt_deg):
        pan_min = self.min_pan_softstop_deg
        pan_max = self.max_pan_softstop_deg
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


  def subscribe_pt_topic(self, topic):
    self.msg_if.pub_warn("subscribe_pt_topic Called")

    success = False
    if self.pt_connect_if is not None:
      success = self.unsubscribe_pt_topic()

    pt_connect_if = ConnectPTXDeviceIF(namespace = topic,
                                       panTiltCb = self.panTiltCb,
                                       stopPanCb = self.stopPanCb,
                                       stopTiltCb = self.stopTiltCb,
                                       msg_if = self.msg_if
                                        )
    ready = pt_connect_if.wait_for_ready()
    if ready == True:
      self.pt_connect_if = pt_connect_if
      self.pt_connected_topic = topic
      self.msg_if.pub_warn("pt_connected_topic: " + str(self.pt_connected_topic))
      self.pt_connect_if.set_speed_ratio(self.speed_ratio)
      self.pt_connect_if.set_pan_speed_ratio(self.pan_speed_ratio)
      self.pt_connect_if.set_tilt_speed_ratio(self.tilt_speed_ratio)
      # Own subscriber to the timestamped PT feedback (NavPosePanTilt) to feed the
      # PT state buffer. The ConnectPTXDeviceIF panTiltCb hook drops the stamp, so
      # subscribe directly. Topic is <pt_namespace>/pan_tilt.
      try:
          if self.pt_pos_sub is not None:
              self.pt_pos_sub.unregister()
      except:
          pass
      self.pt_pos_sub = nepi_sdk.create_subscriber(topic + '/pan_tilt', NavPosePanTilt, self.ptStateCb, queue_size = 1, log_name_list = [])
    return success
  


  
  def unsubscribe_pt_topic(self):
    self.msg_if.pub_warn("unsubscribe_pt_topic Called")

    success = True
    if self.pt_pos_sub is not None:
      try:
          self.pt_pos_sub.unregister()
      except:
          pass
      self.pt_pos_sub = None
    if self.pt_connect_if is not None:
      success = self.pt_connect_if.unregister()
      self.pt_connected = False
      self.pt_connected_topic = None
      self.current_position = None
      nepi_sdk.sleep(1)
      self.pt_connect_if = None
    return success

  def panTiltCb(self, pan_deg, tilt_deg):
     self.current_position = [pan_deg, tilt_deg]
     #self.msg_if.pub_warn("PT position: " + str(self.current_position))

  def ptStateCb(self, msg):
     # Timestamped PT feedback (NavPosePanTilt) -> PT state buffer. Prefer the
     # full-precision ROS header stamp; the driver currently leaves it unset and
     # only fills the float32 .timestamp (~128 s resolution at epoch, unusable),
     # so fall back to arrival time from nepi_sdk.get_time() (same clock, full
     # precision). Auto-upgrades if the driver starts stamping the header.
     t = 0.0
     try:
         t = nepi_sdk.sec_from_msg_stamp(msg.header.stamp)
     except:
         t = 0.0
     if t is None or t <= 0.0:
         t = nepi_sdk.get_time()
     pan_deg = msg.pan_deg
     tilt_deg = msg.tilt_deg
     self.pt_last_timestamp = t
     self.current_position = [pan_deg, tilt_deg]
     if self.pt_state_buffer is not None:
         self.pt_state_buffer.add(t, {'pan_deg': pan_deg, 'tilt_deg': tilt_deg})

  def resolveStateAtTime(self, t_img):
     # Reconstruct platform (IMU + PT) state at a past image timestamp t_img by
     # interpolating the state buffers. Returns a dict the controller can use for
     # latency-correct target-vector geometry; 'valid' is True only when BOTH the
     # IMU and PT buffers bracket t_img. Raw sensor signs are preserved here; any
     # frame/sign convention is applied later in the controller geometry.
     imu = None
     pt = None
     if self.imu_state_buffer is not None:
         imu = self.imu_state_buffer.resolve(t_img)
     if self.pt_state_buffer is not None:
         pt = self.pt_state_buffer.resolve(t_img)
     imu_valid = bool(imu is not None and imu['_valid'])
     pt_valid = bool(pt is not None and pt['_valid'])
     state = {
         't_img': t_img,
         'roll_deg': imu['roll_deg'] if imu_valid else -999,
         'pitch_deg': imu['pitch_deg'] if imu_valid else -999,
         'pan_deg': pt['pan_deg'] if pt_valid else -999,
         'tilt_deg': pt['tilt_deg'] if pt_valid else -999,
         'imu_valid': imu_valid,
         'pt_valid': pt_valid,
         'imu_age_sec': imu['_age_sec'] if imu is not None else -1.0,
         'pt_age_sec': pt['_age_sec'] if pt is not None else -1.0,
         'imu_extrapolated': bool(imu is not None and imu['_extrapolated']),
         'pt_extrapolated': bool(pt is not None and pt['_extrapolated']),
         'valid': imu_valid and pt_valid,
     }
     return state

  def stopPanCb(self):
     pass



  def stopTiltCb(self):
    pass







##########################
###Image Veiwer
##########################

  def mouseClickCb(self,msg):
      #self.msg_if.pub_warn("Got Mouse Click Index " + str(msg.image_index))
      if msg.click_event == True:
        click_count = msg.click_count

        if click_count > 1:
            if self.num_windows == 1:
                if self.has_dual_mode == True:
                    self.setNumWindows(2)
            else:
                image_index = msg.image_index
                image_topic = msg.image_topic
                if (image_index == 1 or image_index == 3) and self.has_zoom_mode == True:
                    #self.msg_if.pub_warn("Setting zoom mode to True")
                    self.zoom_mode_enabled = True
                else:
                    #self.msg_if.pub_warn("Setting zoom mode to False")
                    self.zoom_mode_enabled = False
                self.setNumWindows(1)
            self.publish_status()
            
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
            geom_valid = False
            if image_width > 10 and image_height > 10 and image_fov_horz > 10 and image_fov_vert > 10 and image_zoom_ratio < 0.1:
                object_loc_x_ratio_from_center = float(pixel[0] - image_width/2) / float(image_width/2)
                object_loc_y_ratio_from_center = float(pixel[1] - image_height/2) / float(image_height/2)
                vert_angle_deg = (object_loc_y_ratio_from_center * float(image_fov_vert/2))
                horz_angle_deg = - (object_loc_x_ratio_from_center * float(image_fov_horz/2))
                self.click_position = [horz_angle_deg,vert_angle_deg]
                geom_valid = True

            # When an auto mode (Stabilize/Sweep) is active, a click does NOT go
            # straight to the driver. It routes through the auto controller as a
            # target-vector update -- exactly like an AI detection: same camera-frame
            # offset, same roll/pitch-compensated velocity servo. The controller
            # moves the STAB held vector to the clicked point (and a click during
            # Sweep falls the mode back to Stabilize). Manual-mode clicks keep the
            # legacy direct-to-driver behavior below.
            if (self.stab_enabled or self.scan_enabled):
                if geom_valid:
                    self.click_update = {'az_off': self.click_position[0],
                                         'el_off': self.click_position[1]}
                    self.msg_if.pub_info("Click -> target vector update (az_off %.2f, el_off %.2f)" % (self.click_position[0], self.click_position[1]))
                return

            pan_cur = self.current_position[0]
            pan_to_goal = self.click_position[0] + pan_cur
            self.pan_click_position_udpate = copy.deepcopy(self.click_position[0])
            if click_pan_enabled == True:
                if self.current_position == None:
                    pass
                else:
                    self.msg_if.pub_warn("Pixel Selected, Going to Pan Pos " + str(pan_to_goal))#)
                    self.goto_position[0] = pan_to_goal
                    self.pt_connect_if.goto_to_pan_position(pan_to_goal)
                    
            else: 
                self.msg_if.pub_warn("Pan Click Enabled is False")#)


            tilt_cur = self.current_position[1]
            tilt_to_goal = self.click_position[1] + tilt_cur
            self.tilt_click_position_udpate = copy.deepcopy(self.click_position[1])
            if click_tilt_enabled == True:
                if self.current_position == None:
                    pass
                else:
                    self.goto_position[1] = tilt_to_goal
                    self.msg_if.pub_warn("Pixel Selected, Going to Tilt Pos " + str(tilt_to_goal))
                    self.pt_connect_if.goto_to_tilt_position(tilt_to_goal)
            else: 
                self.msg_if.pub_warn("Tilt Click Enabled is False")#)


  def setImageTopic1Cb(self,msg):
    self.msg_if.pub_info(str(msg))
    img_topic = msg.data.replace('detections_image','color_image')
    if img_topic != 'None':
        self.selected_image_topics[0] = img_topic
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('selected_image_topics', self.selected_image_topics)
            #self.node_if.save_config()

  def setImageTopic2Cb(self,msg):
    self.msg_if.pub_info(str(msg))
    img_topic = msg.data.replace('detections_image','color_image')
    if img_topic != 'None':
        self.selected_image_topics[1] = img_topic
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('selected_image_topics', self.selected_image_topics)
            #self.node_if.save_config()

  def setImageTopic3Cb(self,msg):
    self.msg_if.pub_info(str(msg))
    img_topic = msg.data.replace('detections_image','color_image')
    if img_topic != 'None':
        self.selected_image_topics[2] = img_topic
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('selected_image_topics', self.selected_image_topics)
            #self.node_if.save_config()

  def setImageTopic4Cb(self,msg):
    self.msg_if.pub_info(str(msg))
    img_topic = msg.data.replace('detections_image','color_image')
    if img_topic != 'None':
        self.selected_image_topics[3] = img_topic
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
      self.num_windows = num_windows
      self.publish_status()
      if self.node_if is not None:
        self.node_if.set_param('num_windows', self.num_windows)
        #self.node_if.save_config()

  def setFullScreenEnableCb(self,msg):
    self.msg_if.pub_info(str(msg))
    enabled = msg.data
    self.full_screen_enabled = enabled
    # if self.node_if is not None:
    #     self.node_if.set_param('full_screen_enabled', enabled)
    #     #self.node_if.save_config()

  def setImageStreamQualityCb(self, msg):
    ratio = nepi_utils.check_ratio(msg.data)
    self.stream_quality = ratio
    self.publish_status()
    if self.node_if is not None:
        self.node_if.set_param('stream_quality', self.stream_quality)
        #self.node_if.save_config()

  def setImageStreamRateCb(self, msg):
    ratio = nepi_utils.check_ratio(msg.data)
    self.stream_rate = ratio
    self.publish_status()
    if self.node_if is not None:
        self.node_if.set_param('stream_rate', self.stream_rate)
        #self.node_if.save_config()

  def setImageOveralysEnableCb(self, msg):
    enabled = msg.data
    self.overlays_enabled = enabled
    self.publish_status()
    if self.node_if is not None:
        self.node_if.set_param('overlays_enabled', self.overlays_enabled)
        #self.node_if.save_config()


##########################
### Status
##########################  

 
  

  def publishStatusCb(self,timer):


    self.publish_status()


  def publish_status(self):



    ###############
    ###Images
    ###############
    self.status_msg.full_screen_enabled = self.full_screen_enabled
    self.status_msg.has_dual_mode = self.has_dual_mode
    daul_mode = (self.num_windows > 1)
    self.status_msg.dual_mode_enabled = daul_mode
    self.status_msg.is_night = self.is_night
    self.status_msg.has_night_mode = self.has_night_mode
    night_mode = self.night_mode_enabled
    self.status_msg.night_mode_enabled = night_mode
    self.status_msg.has_zoom_mode = self.has_zoom_mode
    zoom_mode = self.zoom_mode_enabled
    self.status_msg.zoom_mode_enabled = zoom_mode
    detect_mode = self.detect_mode_enabled
    self.status_msg.detect_mode_enabled = detect_mode
    self.status_msg.stream_quality = self.stream_quality
    self.status_msg.stream_rate = self.stream_rate
    self.status_msg.overlays_enabled = self.overlays_enabled

    self.status_msg.num_windows = self.num_windows

    self.status_msg.has_dual_mode = self.has_dual_mode
    self.status_msg.has_zoom_mode = self.has_zoom_mode
    self.status_msg.has_night_mode = self.has_night_mode

    
    
    ###############
    ###Pan Tilt
    ###############


    self.status_msg.available_pan_tilts = self.available_pan_tilts
    selected_pan_tilt = 'None'
    if self.selected_pan_tilt in self.available_pan_tilts:
       selected_pan_tilt = self.selected_pan_tilt
    self.status_msg.selected_pan_tilt = selected_pan_tilt

    pt_connected_topic = self.pt_connected_topic
    if pt_connected_topic is None:
       pt_connected_topic = 'None'
    self.status_msg.pt_connected_topic = pt_connected_topic
    self.status_msg.pt_connected = self.pt_connected

 


    current_position = [-999,-999]
    #self.msg_if.pub_warn("self.current_position: " + str(self.current_position))
    if self.current_position is not None:
      current_position = self.current_position

    pt_status_msg = None
    if self.pt_connect_if is not None:
       pt_status_msg = self.pt_connect_if.get_status_msg()
    
    if pt_status_msg is None:
       pt_status_msg = DevicePTXStatus()
    
    
    self.status_msg.pan_deg = pt_status_msg.pan_now_deg
    self.status_msg.tilt_deg = pt_status_msg.tilt_now_deg

    self.status_msg.pan_goal = pt_status_msg.pan_goal_deg
    self.status_msg.tilt_goal = pt_status_msg.tilt_goal_deg


    self.status_msg.pt_status_msg = pt_status_msg
    self.pan_tilt_max_speed_dps = pt_status_msg.speed_max_dps
    self.status_msg.pan_tilt_max_speed_dps = pt_status_msg.speed_max_dps
    self.pan_deg_per_sec = pt_status_msg.speed_pan_dps
    self.status_msg.pan_deg_per_sec = pt_status_msg.speed_pan_dps
    self.tilt_deg_per_sec = pt_status_msg.speed_tilt_dps
    self.status_msg.tilt_deg_per_sec = pt_status_msg.speed_tilt_dps
    
    self.status_msg.pan_tilt_avg_move_delay = self.pan_tilt_avg_move_delay

    self.status_msg.speed_ratio = pt_status_msg.speed_ratio
    self.status_msg.pan_speed_ratio = pt_status_msg.speed_pan_ratio
    self.status_msg.tilt_speed_ratio = pt_status_msg.speed_tilt_ratio








    



    ###################################
    # Scan
    self.scan_ready = self.pt_connected
    self.status_msg.scan_ready = self.scan_ready

    self.status_msg.scan_pan_enabled = self.scan_pan_enabled


    self.pan_scanning = self.scan_pan_enabled and self.scan_ready
    self.status_msg.pan_scanning = self.pan_scanning
    self.tilt_scanbing = self.scan_tilt_enabled and self.scan_ready
    self.status_msg.tilt_scanning = self.tilt_scanning


    self.status_msg.scan_enabled = self.scan_enabled
    # Per-axis scan deprecated: derive pan/tilt status from the whole-unit Sweep
    # enable so the RUI reflects state and gates sliders correctly.
    self.status_msg.scan_pan_enabled = self.scan_enabled
    self.status_msg.scan_tilt_enabled = self.scan_enabled
    self.status_msg.pan_scanning = self.scan_enabled and self.scan_ready
    self.status_msg.tilt_scanning = self.scan_enabled and self.scan_ready



    ###################################
    # Track
    tracking_dict = copy.deepcopy(self.tracking_dict)
    targets_status_msg = copy.deepcopy(self.targets_status_msg)
    tracking_available_sources = copy.deepcopy(self.tracking_available_sources)

    self.status_msg.available_track_source_namespaces = tracking_available_sources
    track_source_selected = tracking_dict['targets_topic']
    if track_source_selected == 'None' or track_source_selected == '':
        if len(tracking_available_sources) > 0:
            track_source_selected = tracking_available_sources[0]
            self.tracking_dict['targets_topic'] = track_source_selected

    self.status_msg.track_source_selected = tracking_dict['targets_topic']
    self.status_msg.track_source_connected = self.targets_status_msg is not None and self.tracking_targets_connected == True
    self.status_msg.track_ready = self.track_ready

    
    self.status_msg.track_enabled = self.track_enabled
    # Per-axis track deprecated: derive pan/tilt status from the whole-unit
    # AI Tracking enable.
    self.status_msg.track_pan_enabled = self.track_enabled
    self.status_msg.track_tilt_enabled = self.track_enabled

    self.pan_tracking = self.track_enabled and self.track_ready
    self.status_msg.pan_tracking = self.pan_tracking
    self.tilt_trackbing = self.track_enabled and self.track_ready
    self.status_msg.tilt_tracking = self.tilt_trackbing

    self.status_msg.track_pan_error = self.track_pan_error
    self.status_msg.track_tilt_error = self.track_tilt_error

    track_image_topic = tracking_dict['source_topic']
    self.status_msg.track_image_topic = track_image_topic

    track_threshold = tracking_dict['threshold_filter']
    self.status_msg.track_threshold = track_threshold

    self.status_msg.track_best_filter_options = self.tracking_best_filter_options
    self.status_msg.track_best_filter = tracking_dict['best_filter']
    
    if targets_status_msg is not None:
        track_image_sources = targets_status_msg.process_status.available_source_topics
        if track_image_topic in track_image_sources and track_image_topic != 'None':
            self.sendTargetsMsg('source_pub',track_image_topic)
        if round(track_threshold,2) != round(targets_status_msg.threshold_filter,2):
            #self.msg_if.pub_warn("Status sending threshold update: " + str(threshold))
            self.sendTargetsMsg('threshold_pub',track_threshold)


    ###################################
    # Stab
    available_stab_source_dict = copy.deepcopy(self.available_stab_source_dict)
    available_stab_source_namespaces =  list(available_stab_source_dict.keys())
    self.status_msg.available_stab_source_namespaces = available_stab_source_namespaces
    selected_stab_source = self.selected_stab_source
    if selected_stab_source not in available_stab_source_namespaces:
        selected_stab_source = 'None'

    self.status_msg.selected_stab_source = selected_stab_source
    self.status_msg.stab_source_connected = self.stab_source_connected
    
    self.status_msg.stab_ready = self.stab_ready
    
    self.status_msg.stab_enabled = self.stab_enabled
    # Per-axis stab deprecated: derive pan/tilt status from the whole-unit
    # Stabilize enable.
    self.status_msg.stab_pan_enabled = self.stab_enabled
    self.status_msg.stab_tilt_enabled = self.stab_enabled

    self.pan_stabbing = self.stab_enabled and self.stab_ready
    self.status_msg.pan_stabbing = self.pan_stabbing
    self.tilt_stabbing = self.stab_enabled and self.stab_ready
    self.status_msg.tilt_stabbing = self.tilt_stabbing

    self.status_msg.has_image_stab = self.stab_ready
    self.status_msg.image_stab_enabled = self.image_stab_enabled

    ###################################
    # Auto
    auto_data_dict = copy.deepcopy(self.auto_data_dict)
    try:
        self.status_msg.pan_control_disabled = self.status_msg.pan_scanning or self.status_msg.pan_tracking or self.status_msg.pan_stabbing
        self.status_msg.tilt_control_disabled = self.status_msg.tilt_scanning or self.status_msg.tilt_tracking or self.status_msg.tilt_stabbing

        self.status_msg.auto_pan_min_deg = round(self.auto_pan_min_deg, 0)
        self.status_msg.auto_pan_max_deg = round(self.auto_pan_max_deg, 0)
        self.status_msg.auto_tilt_min_deg = round(self.auto_tilt_min_deg, 0)
        self.status_msg.auto_tilt_max_deg = round(self.auto_tilt_max_deg, 0)

        goto_pos = copy.deepcopy(self.goto_position)
        self.status_msg.pan_goto = goto_pos[0]
        self.status_msg.tilt_goto = goto_pos[1]

        self.status_msg.auto_lat = auto_data_dict['auto_lat']
        self.status_msg.auto_long = auto_data_dict['auto_long']


        self.status_msg.auto_pan_pos = auto_data_dict['auto_pan_pos']
        self.status_msg.auto_tilt_pos = auto_data_dict['auto_tilt_pos']
        self.status_msg.auto_pos_display_title = str(auto_data_dict['auto_pos_display_title'])
        self.status_msg.auto_pan_pos_display = auto_data_dict['auto_pan_pos_display']
        self.status_msg.auto_tilt_pos_display = auto_data_dict['auto_tilt_pos_display']
        self.status_msg.auto_pan_pos_disabled = auto_data_dict['auto_pan_pos_disabled']
        self.status_msg.auto_tilt_pos_disabled = auto_data_dict['auto_tilt_pos_disabled']

        self.status_msg.auto_pan_ratio_set = auto_data_dict['auto_pan_ratio_set']
        self.status_msg.auto_tilt_ratio_set = auto_data_dict['auto_tilt_ratio_set']
        self.status_msg.auto_pan_ratio_display = auto_data_dict['auto_pan_ratio_display']
        self.status_msg.auto_tilt_ratio_display = auto_data_dict['auto_tilt_ratio_display']
        # Manual mode: the auto controller is idle, so auto_*_ratio_set keeps its 0.5
        # default and the RUI slider thumb sits centered. Drive the thumb from the live
        # driver feedback (pan/tilt_now_ratio, already in the same reverse-corrected
        # frame as goto_pan_ratio) so the indicator shows the real PT position.
        if self.is_manual_mode():
            self.status_msg.auto_pan_ratio_set = pt_status_msg.pan_now_ratio
            self.status_msg.auto_tilt_ratio_set = pt_status_msg.tilt_now_ratio
            self.status_msg.auto_pan_ratio_display = pt_status_msg.pan_now_ratio
            self.status_msg.auto_tilt_ratio_display = pt_status_msg.tilt_now_ratio
        self.status_msg.auto_pan_ratio_disabled = auto_data_dict['auto_pan_ratio_disabled']
        self.status_msg.auto_tilt_ratio_disabled = auto_data_dict['auto_tilt_ratio_disabled']

        self.status_msg.auto_pan_speed_ratio_set = auto_data_dict['auto_pan_speed_ratio_set']
        self.status_msg.auto_tilt_speed_ratio_set = auto_data_dict['auto_tilt_speed_ratio_set']
        self.status_msg.auto_pan_speed_disabled = auto_data_dict['auto_pan_speed_ratio_disabled']
        self.status_msg.auto_tilt_speed_disabled = auto_data_dict['auto_tilt_speed_ratio_disabled']
    except:
        pass
    available_auto_processes =  self.available_auto_processes
    self.status_msg.available_auto_processes = available_auto_processes
    selected_auto_process = self.selected_auto_process
    if selected_auto_process not in available_auto_processes:
        selected_auto_process = 'None'

    self.status_msg.selected_auto_process = selected_auto_process
    self.status_msg.auto_process_ready = self.auto_process_ready
    


    self.status_msg.auto_night_enabled = self.auto_night_mode_enabled


    self.status_msg.roll_deg = self.auto_data_dict['roll_deg']
    self.status_msg.roll_dps = self.auto_data_dict['roll_dps']

    self.status_msg.pitch_deg = self.auto_data_dict['pitch_deg']
    self.status_msg.pitch_dps = self.auto_data_dict['pitch_dps']

    self.status_msg.yaw_deg = self.auto_data_dict['yaw_deg']
    self.status_msg.yaw_dps = self.auto_data_dict['yaw_dps']

    self.status_msg.heading_deg = self.auto_data_dict['yaw_deg']
    self.status_msg.heading_dps = self.auto_data_dict['yaw_dps']
 
    self.status_msg.auto_pan_deg = self.auto_data_dict['auto_pan_deg']
    self.status_msg.auto_pan_dps = self.auto_data_dict['auto_pan_dps']
    self.status_msg.auto_pan_adj = self.auto_data_dict['auto_pan_adj']
    self.status_msg.auto_pan_goal = self.auto_data_dict['auto_pan_goal']
    self.status_msg.auto_pan_pos_rate = self.auto_data_dict['auto_pan_pos_rate']
    self.status_msg.auto_pan_vel_rate = self.auto_data_dict['auto_pan_vel_rate']
    

    self.status_msg.auto_tilt_deg = self.auto_data_dict['auto_tilt_deg']
    self.status_msg.auto_tilt_dps = self.auto_data_dict['auto_tilt_dps']
    self.status_msg.auto_tilt_adj = self.auto_data_dict['auto_tilt_adj']
    self.status_msg.auto_tilt_goal = self.auto_data_dict['auto_tilt_goal']
    self.status_msg.auto_tilt_pos_rate = self.auto_data_dict['auto_tilt_pos_rate']
    self.status_msg.auto_tilt_vel_rate = self.auto_data_dict['auto_tilt_vel_rate']
    



    ###########
    if self.node_if is not None:
      if self.status_has_published == False:
        #self.msg_if.pub_warn("Publishing Status: " + str(self.status_msg))
        self.status_has_published = True
      self.node_if.publish_pub('status_pub', self.status_msg) 
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
    
    track_status_sources = nepi_sdk.find_topics_by_msg('TargetingStatus', topics_list = self.active_topics, types_list = self.active_topic_types)
    track_sources = []
    for i, topic in enumerate(track_status_sources):
        track_sources.append(topic.replace('/status',''))
    self.tracking_available_sources = track_sources

    # self.msg_if.pub_warn("")
    # self.msg_if.pub_warn("")
    # self.msg_if.pub_warn("")
    # self.msg_if.pub_warn("Starting Tracking Updater Purge Check")
    # self.msg_if.pub_warn("available targets: " + str(self.tracking_available_sources))
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
    # self.msg_if.pub_warn("available targets: " + str(self.tracking_available_sources))
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
            if last_time > AUTO_SOURCE_RESET_TIMEOUT_SEC:
                self.msg_if.pub_warn("Clearing targets status message on timeout")
                self.track_ready = False
                self.targets_status_msg = None
                self.targets_status_last_time = None
                self.tracking_running = False
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
            #tracking_subpub_dict['class_pub'] = nepi_sdk.create_publisher(namespace + '/set_class', String, queue_size = 10, log_name_list = [])
            tracking_subpub_dict['threshold_pub'] = nepi_sdk.create_publisher(namespace + '/set_threshold', Float32, queue_size = 10, log_name_list = [])
            #tracking_subpub_dict['save_pub'] = nepi_sdk.create_publisher(namespace + '/set_save_config_enable', Bool, queue_size = 10, log_name_list = [])
            #tracking_subpub_dict['config_pub'] = nepi_sdk.create_publisher(namespace + '/set_config', TargetingUpdate, queue_size = 10, log_name_list = [])

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



  def filter_by_range_angles(self,targets_dict_list):
    ################
    # Filter by min max range and angles
    filtered_dict_list = []
    cur_position = copy.deepcopy(self.current_position)
    if cur_position is not None:
      [cur_pan,cur_tilt] = [cur_position[0],cur_position[1]]
      range_min = self.track_range_min_m
      range_max = self.track_range_max_m
      pan_min = self.auto_pan_min_deg #track_pan_min_deg
      pan_max = self.auto_pan_max_deg #track_pan_max_deg
      tilt_min = self.auto_tilt_min_deg #track_tilt_min_deg
      tilt_max = self.auto_tilt_max_deg #track_tilt_max_deg

      for target_dict in targets_dict_list:
          target_valid = True
          range_m = target_dict['range_m']
          if (range_m < range_min or range_m > range_max) and range_m != -999:
            target_valid = False
          target_pan_angle = target_dict['azimuth_deg']
          pan_angle =  cur_pan + target_pan_angle
          if (pan_angle < pan_min or pan_angle > pan_max) and target_pan_angle != -999:
            target_valid = False
          target_tilt_angle = target_dict['elevation_deg']
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




  def customFilterCb(self,targets_dict_list):
     filtered_dict_list = self.filter_by_range_angles(targets_dict_list)
     return filtered_dict_list


  def targetsCb(self,msg, args):
    #self.msg_if.pub_info("Targets callback got new targets mgs: " + str(msg))
    targets_namespace = args
    self.targets_last_time = nepi_utils.get_time()

    self.targets_msg = msg.targets
    timestamp = msg.source_timestamp
    #self.msg_if.pub_info("Got targets mgs: " + str(self.targets_msg))
    tracking_dict = copy.deepcopy(self.tracking_dict)
    targets_topic = tracking_dict['targets_topic']
    source_topic = tracking_dict['source_topic']
    if True: #targets_topic == msg.process_namespace and source_topic == msg.source_topic:
        self.targets_last_time = nepi_utils.get_time()


        #self.msg_if.pub_warn("Got targets msg list " + str(targets_msg))
        # Reconstruct the platform (IMU + PT) state at the detection image
        # time ONCE for this frame; every target in the message shares it.
        # The auto controller's TRACK/LOCK ingest REQUIRES
        # target_dict['state_at_img'] (it rejects any detection without a
        # valid one) and it reads target_dict['timestamp']; both are
        # attached here. Without state_at_img the lock never engages and the
        # target vector is never updated by detections.
        if timestamp is None or timestamp <= 0.0:
            # Detector left source_timestamp unset -> use arrival time so the
            # state-buffer resolve gets a positive query on the feeders' clock.
            timestamp = nepi_utils.get_time()
        state_at_img = self.resolveStateAtTime(timestamp)
        targets_dict_list = []
        for target_msg in self.targets_msg:
            target_dict = nepi_targets.convert_target_msg2dict(target_msg)
            if target_dict is not None:
                target_dict['timestamp'] = timestamp
                target_dict['state_at_img'] = state_at_img
                targets_dict_list.append(target_dict)
            #self.msg_if.pub_warn("Added target list for name " + str(target_dict['target_name']))
        #self.msg_if.pub_warn("Got targets list " + str(targets_dict_list))
        self.targets_list = targets_dict_list
        self.targets_timestamp = timestamp



  def targetsStatusCb(self,msg, args):
    #self.msg_if.pub_info("Targets callback got new targets mgs")
    targets_namespace = args
    status_msg = msg
    self.track_ready = True
    if self.targets_status_msg_start is None:
        #self.msg_if.pub_warn("Captured current Status msg from Targers namespace: " + str(namespace) + " : " + str(msg))
        self.targets_status_msg_start = msg
    self.tracking_running = True
    self.tracking_targets_connecting = False
    self.tracking_targets_connected = True
    self.targets_status_msg = msg

    self.targets_status_last_time = nepi_utils.get_time()



 

##########################
### Stab
##########################  



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


  def setStabCb(self, msg):
        enabled = msg.data
        self.msg_if.pub_info("Setting stab: " + str(enabled))
        self.setStab(enabled)


  def setStab(self,enabled):
        self.stab_enabled = enabled
        if enabled and self.scan_enabled:
            # Stabilize and Sweep are mutually exclusive.
            self.scan_enabled = False
            if self.node_if is not None:
                self.node_if.set_param('scan_enabled', self.scan_enabled)
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('stab_enabled', self.stab_enabled)


  def setStabPanCb(self, msg):
        enabled = msg.data
        self.msg_if.pub_info("Setting stab pan: " + str(enabled))
        self.setStabPan(enabled)


  def setStabPan(self,enabled):
        self.stab_pan_enabled = enabled
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('stab_pan_enabled', self.stab_pan_enabled)

  def setStabTiltCb(self, msg):
        enabled = msg.data
        self.msg_if.pub_info("Setting stab tilt: " + str(enabled))
        self.setStabTilt(enabled)

  def setStabTilt(self,enabled):
        self.stab_tilt_enabled = enabled
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('stab_tilt_enabled', self.stab_tilt_enabled)

  def setStabPanPosRatioCb(self, msg):
      self.setStabPanPosRatio(msg.data)

  def setStabPanPosRatio(self, pos_ratio):
        # Image-viewer Pan slider. The RUI publishes here in EVERY mode (its
        # goto_pan_ratio direct-to-driver path is overridden to this topic whenever
        # app status is present). So branch on mode:
        #  - AUTO (Stabilize/Sweep): forward the raw 0-1 ratio to the controller as a
        #    one-shot; the controller owns the az ruler (pan soft-limit span) and the
        #    Sweep->Stabilize fallback (pt_auto_2 'Slider drag' block, gated on
        #    pan_stab/pan_scan -> it IGNORES the ratio in manual mode).
        #  - MANUAL: controller is in HOLD and drops the ratio, so drive the axis
        #    directly. goto_pan_ratio hits the same driver topic the RUI would use
        #    bypassing the app (ratio->position mapping, softstops + reverse all
        #    handled in the driver) = identical to the legacy manual slider.
        pos_ratio = nepi_utils.check_ratio(pos_ratio)
        if self.is_manual_mode():
            if self.pt_connect_if is not None:
                self.pt_connect_if.goto_pan_ratio(pos_ratio)
        else:
            self.pan_ratio_update = pos_ratio

  def setStabTiltPosRatioCb(self, msg):
      self.setStabTiltPosRatio(msg.data)

  def setStabTiltPosRatio(self, pos_ratio):
        # Image-viewer Tilt slider. Same dual path as the pan slider above:
        #  - AUTO: forward to the controller (owns the el ruler + tilt inversion;
        #    in Sweep this retunes the sweep elevation in place).
        #  - MANUAL: controller in HOLD ignores it -> direct goto_tilt_ratio to the
        #    driver (identical to the legacy manual slider).
        pos_ratio = nepi_utils.check_ratio(pos_ratio)
        if self.is_manual_mode():
            if self.pt_connect_if is not None:
                self.pt_connect_if.goto_tilt_ratio(pos_ratio)
        else:
            self.tilt_ratio_update = pos_ratio

  def checkForStabSourceTopic(self,namespace):
      check_topic = namespace
      found = check_topic in list(self.available_stab_source_dict.keys())
      return found

  def updaterStabCb(self,timer):
    #self.msg_if.pub_warn("Stab Updater Called")

    needs_publish = False
    avail_stab_sources = []
    avail_stab_sources_dict = dict()
    for message in nepi_auto_pt.NAVPOSE_SOURCE_MESSAGE_DICT.keys():
        avail_sources = nepi_sdk.find_topics_by_msg(message,self.active_topics,self.active_topic_types)
        for source in avail_sources:
            if message != 'NavPose' or (message == 'NavPose' and 'navposes' in source and os.path.basename(source) == 'navpose'):
                avail_stab_sources.append(source)
                avail_stab_sources_dict[source] = nepi_auto_pt.NAVPOSE_SOURCE_MESSAGE_DICT[message]              
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
            if self.AUTO_DEFAULT_SOURCE in source:
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
    if self.stab_source_connected == True:
        if self.last_stab_source_time is not None:
            last_time = nepi_utils.get_time() - self.last_stab_source_time
            if last_time > AUTO_SOURCE_RESET_TIMEOUT_SEC:
                self.msg_if.pub_warn("Clearing stab status message on timeout")

                self.auto_dict_lock.acquire()
                self.auto_data_dict = nepi_auto_pt.get_blank_data_dict()
                self.auto_dict_lock.release()
                self.stab_ready = False
                self.stab_pan_adj = 0.0
                self.stab_tilt_adj = 0.0
                self.last_stab_source_time = None

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
            self.msg_if.pub_warn("Subscribing to NavPose topic: " + str(namespace))
            stab_subpub_dict = dict()

            message = self.available_stab_source_dict[namespace]
            stab_subpub_dict['source_sub'] = nepi_sdk.create_subscriber(namespace, message, self.stabSourceCb, queue_size = 1, log_name_list = [])
            config_namespace = os.path.dirname(namespace) + '/config'
            self.msg_if.pub_warn("Subscribing to NavPose Config: " + str(config_namespace))
            stab_subpub_dict['config_sub'] = nepi_sdk.create_subscriber(config_namespace, NavPoseSolution, self.stabSourceConfigCb, queue_size = 1, log_name_list = [])

            self.stab_subpub_lock.acquire()
            self.stab_subpub_dict = stab_subpub_dict
            nepi_sdk.sleep(1)
            self.stab_subpub_lock.release()

            self.stab_source_connected = False
            self.stab_source_connected_message = message

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
        self.stab_source_connected_message = None
        self.last_stab_source_time = None
        self.navpose_config = None
        return True


  def goto_to_pan_position_adj(self,pan_deg):
    if self.stab_pan_enabled == True:
        adj_pan_goal = pan_deg + self.stab_pan_adj
        self.pt_connect_if.goto_to_pan_position(adj_pan_goal)

  def goto_to_tilt_position_adj(self,tilt_deg):
    if self.stab_tilt_enabled == True:
        adj_tilt_goal = tilt_deg + self.stab_tilt_adj
        self.pt_connect_if.goto_to_tilt_position(adj_tilt_goal)



  def stabSourceConfigCb(self, msg):
      #self.msg_if.pub_warn("Got NavPose Config msg " + str(msg), throttle_s = 10)
      self.navpose_config = nepi_sdk.convert_msg2dict(msg)


  def stabSourceCb(self,msg):
    #self.msg_if.pub_warn("******")
    #self.msg_if.pub_warn("*** Stabs Source Update Starting ***")
    #self.msg_if.pub_warn("******", throttle_s=1)
    self.stab_source_connecting = False
    self.stab_source_connected = True
    self.stab_ready = True
    navpose_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
    navpose_message = self.stab_source_connected_message
    stab_source_dict = nepi_sdk.convert_msg2dict(msg)
    for key in navpose_dict.keys():
        if key in stab_source_dict.keys():
            navpose_dict[key] = stab_source_dict[key]
    if 'timestamp' in stab_source_dict.keys():
        navpose_dict['timestamp'] = stab_source_dict['timestamp']
    
    self.stab_source_lock.acquire()
    self.navpose_dict = navpose_dict
    self.stab_source_dict = stab_source_dict
    self.stab_source_lock.release()

    # Feed the IMU state buffer for image-time state reconstruction. Use the
    # full-precision ROS header stamp; fall back to arrival time (same clock).
    # time_orientation is float32 (~128 s resolution at epoch) so it is not used.
    # Raw sensor roll/pitch are stored; sign/frame conventions belong to the
    # controller geometry, not the buffer.
    imu_t = 0.0
    try:
        imu_t = nepi_sdk.sec_from_msg_stamp(msg.header.stamp)
    except:
        imu_t = 0.0
    if imu_t is None or imu_t <= 0.0:
        imu_t = nepi_utils.get_time()
    if self.imu_state_buffer is not None and 'roll_deg' in stab_source_dict and 'pitch_deg' in stab_source_dict:
        roll_deg = stab_source_dict['roll_deg']
        pitch_deg = stab_source_dict['pitch_deg']
        if roll_deg != -999 and pitch_deg != -999:
            self.imu_state_buffer.add(imu_t, {'roll_deg': roll_deg, 'pitch_deg': pitch_deg})

    last_stab_source_time = copy.deepcopy(self.last_stab_source_time)
    if last_stab_source_time is None:
        last_stab_source_time = 0
    cur_time = nepi_utils.get_time()
    self.last_stab_source_time = cur_time



##########################
### Auto
##########################  

  def setAutoUpdateRateCb(self, msg):
      rate = msg.data
      self.setAutoUpdateRate(rate)

  def setAutoUpdateRate(self, rate):
        if rate < 0:
            rate = 1
        rate = round(rate,1)
        self.msg_if.pub_info("Setting auto update rate to: " + str(rate))
        self.auto_processes_dict[self.selected_auto_process]['auto_update_rate'] = rate
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('auto_processes_dict', self.auto_processes_dict)
            #self.node_if.save_config()

  def setAutoControlCb(self, msg):
      self.msg_if.pub_info("Got Auto Control update message " + str(msg))
      control = msg.name
      value = msg.value
      self.setAutoControl(control,value)

  def setAutoControl(self, control,value):
        auto_process = self.selected_auto_process
        auto_controls_dict = self.auto_processes_dict[auto_process]['auto_controls_dict']
        if control in auto_controls_dict.keys():
            self.msg_if.pub_info("Setting auto control " + str(control) + " : " + str(value))
            auto_controls_dict[control] = value
            self.auto_processes_dict[auto_process]['auto_controls_dict'] = auto_controls_dict
            self.publish_status()
            if self.node_if is not None:
                self.node_if.set_param('auto_processes_dict', self.auto_processes_dict)
                #self.node_if.save_config()



  def reloadAutosCb(self,msg):
    self.auto_process_ready = False
    nepi_sdk.sleep(1)
    try:
        importlib.reload(nepi_auto_pt)
        self.auto_processes_dict = nepi_auto_pt.update_processes_dict(self.auto_processes_dict)
        auto_processes = list(self.auto_processes_dict.keys())
        if self.selected_auto_process not in auto_processes:
            self.selected_auto_process = auto_processes[0]
        self.msg_if.pub_info("Autos reloaded")
        self.auto_process_ready = True
    except Exception as e:
        self.msg_if.pub_info("Failed to reload auto module: " + str(e)) 



  def setAutoProcessCb(self, msg):
      value = msg.data
      self.setAutoProcess(value)

  def setAutoProcess(self,value):
        self.msg_if.pub_info("Setting auto process topic to: " + str(value))
        if value in self.auto_processes_dict.keys():
            self.selected_auto_process = value
            self.publish_status()
            if self.node_if is not None:
                self.node_if.set_param('selected_auto_process', self.selected_auto_process)
                #self.node_if.save_config()



  def _autoLoop(self):
    self.msg_if.pub_warn("Entering AutoLoop Function")
    # Dedicated fixed-rate control thread. Single thread (no per-cycle timer
    # churn), monotonic fixed-grid pacing (immune to NTP wall-clock steps), and a
    # whole-step crash guard so a callback exception can never stop the loop.
    RATE_MIN_HZ = 1.0
    RATE_MAX_HZ = 30.0
    RATE_DEFAULT_HZ = 15.0
    deadline = time.monotonic() + (1.0 / RATE_DEFAULT_HZ)
    while not nepi_sdk.is_shutdown() and self._auto_loop_stop == False:
        # Live rate: clamp every cycle so set_auto_update_rate takes effect on the
        # next tick without recreating anything.
        try:
            sel = self.selected_auto_process
            hz = float(self.auto_processes_dict[sel]['auto_update_rate'])
        except Exception:
            hz = RATE_DEFAULT_HZ
        if hz != hz or hz <= 0.0:   # NaN / non-positive -> default
            hz = RATE_DEFAULT_HZ
        hz = max(RATE_MIN_HZ, min(RATE_MAX_HZ, hz))
        period = 1.0 / hz

        # Gate: until the auto process and PT are ready, idle at the loop rate
        # (skip the heavy per-cycle work, including the deepcopy).
        if not (self.auto_process_ready): #and self.pt_connected and self.pan_tilt_max_speed_dps != -999):
            nepi_sdk.sleep(period)
            deadline = time.monotonic() + period
            continue

        t0 = time.monotonic()
        try:
            self._autoStep()
        except Exception:
            import traceback
            self.msg_if.pub_warn("Auto control step raised (loop continues):\n" + traceback.format_exc(), throttle_s=5.0)
        cycle_s = time.monotonic() - t0
        self._auto_last_cycle_ms = cycle_s * 1000.0
        self._auto_last_overrun = 1 if cycle_s > period else 0

        # Fixed-grid pacing; on overrun resync the grid (no catch-up burst).
        now = time.monotonic()
        sleep_for = deadline - now
        if sleep_for > 0.0:
            nepi_sdk.sleep(sleep_for)
            deadline += period
        else:
            self.msg_if.pub_warn("Auto control cycle overran: %.1f ms > %.1f ms period" % (cycle_s * 1000.0, period * 1000.0), throttle_s=5.0)
            deadline = now + period


  def _autoStep(self):
    #self.msg_if.pub_warn("Entering AutoStep Function")
    selected_auto_process = copy.deepcopy(self.selected_auto_process)
    auto_settings_dict = copy.deepcopy(self.auto_processes_dict[selected_auto_process])

    self.auto_dict_lock.acquire()
    # Reference-swap: reuse the dict produced last cycle as `last` (the controller
    # only reads it) instead of a second deepcopy; build one working copy.
    auto_data_dict_last = self._auto_data_dict_prev
    auto_data_dict = copy.deepcopy(self.auto_data_dict)
    self.auto_dict_lock.release()
    # Carry last cycle's measured timing into this cycle's telemetry payload.
    auto_data_dict['auto2_cycle_ms'] = self._auto_last_cycle_ms
    auto_data_dict['auto2_overrun'] = self._auto_last_overrun



    pt_status_msg = None
    [pan_deg,tilt_deg] = [0,0]
    if self.pt_connect_if is not None:
        pt_status_msg = self.pt_connect_if.get_status_msg()
        if pt_status_msg is not None:
            [pan_raw,tilt_raw] = self.pt_connect_if.get_pan_tilt_position()
            # Reject non-physical position samples (driver feedback glitch): the
            # head cannot be outside its hardstops, so a reading well past the soft
            # limits is bogus. Feeding it to the controller fires a phantom
            # max-speed error blip (the ~430 deg tilt spike). Validate each axis
            # against its soft limits (+ a generous margin) and on a bad sample
            # HOLD the previous good value already in auto_data_dict.
            pan_lo = self.min_pan_softstop_deg - self.PT_POS_REJECT_MARGIN_DEG
            pan_hi = self.max_pan_softstop_deg + self.PT_POS_REJECT_MARGIN_DEG
            tilt_lo = self.min_tilt_softstop_deg - self.PT_POS_REJECT_MARGIN_DEG
            tilt_hi = self.max_tilt_softstop_deg + self.PT_POS_REJECT_MARGIN_DEG
            if pan_lo <= pan_raw <= pan_hi:
                pan_deg = pan_raw
                auto_data_dict['pan_deg'] = pan_deg
                auto_data_dict['pan_timestamp'] = self.pt_last_timestamp
            else:
                pan_deg = auto_data_dict.get('pan_deg', 0.0)
                self.msg_if.pub_warn("Rejected non-physical PT pan sample (deg): " + str(pan_raw), throttle_s=2.0)
            if tilt_lo <= tilt_raw <= tilt_hi:
                tilt_deg = tilt_raw
                auto_data_dict['tilt_deg'] = tilt_deg
                auto_data_dict['tilt_timestamp'] = self.pt_last_timestamp
            else:
                tilt_deg = auto_data_dict.get('tilt_deg', 0.0)
                self.msg_if.pub_warn("Rejected non-physical PT tilt sample (deg): " + str(tilt_raw), throttle_s=2.0)


    # Update auto goals if no auto mode (scan/track/stab) owns the axis
    auto_pan_enabled = self.scan_pan_enabled or self.track_pan_enabled or self.stab_pan_enabled
    auto_tilt_enabled = self.scan_tilt_enabled or self.track_tilt_enabled or self.stab_tilt_enabled



    #############################
    # Update Time Data
    #############################
    cur_time = nepi_utils.get_time()
    auto_data_dict['data_time'] = cur_time
    auto_data_dict['process_time'] = cur_time
    last_time = copy.deepcopy(auto_data_dict['last_auto_time'])
                
    if last_time is None or auto_data_dict_last is None:
            self.msg_if.pub_warn("Auto process got bad time: " + str(last_time) )
            #self.msg_if.pub_warn("Auto process got bad data: " + str(auto_data_dict_last) )
            pass
    else:
        

        #############################
        # Update Pan Tilt Data
        #############################

        if pt_status_msg is not None:
            pan_tilt_max_speed_dps = pt_status_msg.speed_max_dps
            auto_data_dict['pan_tilt_max_speed_dps'] = pan_tilt_max_speed_dps
            auto_data_dict['pan_dps'] = pt_status_msg.speed_pan_dps
            auto_data_dict['tilt_dps'] = pt_status_msg.speed_tilt_dps
        
        auto_data_dict['pan_speed_start_ratio'] = self.auto_pan_speed_start
        auto_data_dict['tilt_speed_start_ratio'] = self.auto_tilt_speed_start

        auto_data_dict['pan_min_deg'] = self.auto_pan_min_deg
        auto_data_dict['pan_max_deg'] = self.auto_pan_max_deg
        auto_data_dict['tilt_min_deg'] = self.auto_tilt_min_deg
        auto_data_dict['tilt_max_deg'] = self.auto_tilt_max_deg


        #############################
        # Update Nav Data
        #############################

        self.stab_source_lock.acquire()
        auto_data_dict['navpose_data'] = self.navpose_dict
        stab_source_dict = self.stab_source_dict
        self.stab_source_lock.release()


        #############################
        # Update Track Data
        #############################
        auto_data_dict['tracking_dict'] = copy.deepcopy(self.tracking_dict)
        auto_data_dict['targets_list'] = copy.deepcopy(self.targets_list)
        targets_timestamp = copy.deepcopy(self.targets_timestamp)
        if auto_data_dict['targets_list'] is not None:
            auto_data_dict['targets_timestamp'] = targets_timestamp
        else:
            auto_data_dict['targets_timestamp'] = 0
        self.targets_list = None


        #############################
        # Update Auto Dict
        #############################

        self.auto_dict_lock.acquire()
        self.auto_data_dict = auto_data_dict
        self.auto_dict_lock.release()


        ##########################
        # Run Auto Process
        ##########################
        if self.auto_process_ready == False:
            self.msg_if.pub_warn("Auto Process Not Ready: " + str(selected_auto_process), throttle_s = 10)  
        else:
            # self.stab_count += 1
            # self.msg_if.pub_warn("Stab_count: " + str(self.stab_count))                 

            auto_data_dict['auto_pt_stop'] = copy.deepcopy(self.pt_stop)
            self.pt_stop = False
            
            auto_data_dict['auto_pan_home'] = copy.deepcopy(self.pan_home)
            self.pan_home = False
            
            auto_data_dict['auto_tilt_home'] = copy.deepcopy(self.tilt_home)
            self.tilt_home = False
            
            auto_data_dict['auto_pan_pos_update'] = copy.deepcopy(self.pan_pos_update)
            self.pan_pos_update = None
            auto_data_dict['auto_tilt_pos_update'] = copy.deepcopy(self.tilt_pos_update)
            self.tilt_pos_update = None

            auto_data_dict['auto_pan_ratio_update'] = copy.deepcopy(self.pan_ratio_update)
            self.pan_ratio_update = None
            auto_data_dict['auto_tilt_ratio_update'] = copy.deepcopy(self.tilt_ratio_update)
            self.tilt_ratio_update = None
            auto_data_dict['auto2_click_update'] = copy.deepcopy(self.click_update)
            self.click_update = None


            auto_data_dict['auto_pan_speed_ratio_update'] = copy.deepcopy(self.pan_speed_ratio_update)
            self.pan_speed_ratio_update = None
            auto_data_dict['auto_tilt_speed_ratio_update'] = copy.deepcopy(self.tilt_speed_ratio_update)
            self.tilt_speed_ratio_update = None


            auto_data_dict['auto_click_pan_pos_update'] = copy.deepcopy(self.pan_click_position_udpate)
            self.pan_click_position_udpate = None
            auto_data_dict['auto_tilt_click_pos_update'] = copy.deepcopy(self.pan_click_position_udpate)
            self.pan_click_position_udpate = None
           
            # --- Whole-unit mode enables (Stabilize / Sweep / AI Tracking) ---
            # pt_auto_2 selects its operating mode from the pan_*_enabled keys,
            # so drive those (and their tilt mirrors) straight from the three
            # whole-unit RUI enables. The tilt axis always rides the active
            # mode's look vector, so the legacy per-axis self.*_pan/tilt_enabled
            # flags no longer gate the controller.
            auto_data_dict['scan_enabled'] = self.scan_enabled
            auto_data_dict['pan_scan_enabled'] = self.scan_enabled
            auto_data_dict['tilt_scan_enabled'] = self.scan_enabled

            auto_data_dict['track_enabled'] = self.track_enabled and self.track_ready
            auto_data_dict['pan_track_enabled'] = self.track_enabled and self.track_ready
            auto_data_dict['tilt_track_enabled'] = self.track_enabled and self.track_ready

            auto_data_dict['stab_enabled'] = self.stab_enabled and self.stab_ready
            auto_data_dict['pan_stab_enabled'] = self.stab_enabled and self.stab_ready
            auto_data_dict['tilt_stab_enabled'] = self.stab_enabled and self.stab_ready
            auto_data_dict['stab_image_enabled'] = self.image_stab_enabled and self.stab_ready

            auto_data_dict['navpose_topic'] = self.stab_source_connected_namespace
            auto_data_dict['navpose_data'] = copy.deepcopy(self.navpose_dict)
            auto_data_dict['navpose_config'] = copy.deepcopy(self.navpose_config)
            auto_data_dict['tracking_dict'] = copy.deepcopy(self.tracking_dict)

            # Snapshot the mode enables we are FEEDING the controller this cycle.
            # The read-back below only absorbs flags the controller CHANGED vs.
            # these inputs -- see the toggle-bounce note there.
            _in_scan_enabled = auto_data_dict['scan_enabled']
            _in_track_enabled = auto_data_dict['track_enabled']
            _in_stab_enabled = auto_data_dict['stab_enabled']

            if selected_auto_process not in nepi_auto_pt.PROCESSES_DICT.keys():
                self.msg_if.pub_warn("Auto Process Not In Proccesses Dict: " + str(selected_auto_process) + " : " + str(nepi_auto_pt.PROCESSES_DICT.keys()), throttle_s = 10) 
            else:
                # self.msg_if.pub_warn("Passing Auto Data Dict: " + str(auto_data_dict), throttle_s = 10)
                auto_process_function = nepi_auto_pt.PROCESSES_DICT[selected_auto_process]['process_function']
                #self.msg_if.pub_warn("Calling auto process function: " + str(auto_process_function), throttle_s = 10)
                try:  # PTAUTO_DIAG temp
                    [auto_data_dict, auto_settings_dict] = auto_process_function(self.pt_connect_if, 
                                                                                        self.images_all_if,
                                                                                        auto_data_dict,
                                                                                        auto_data_dict_last, 
                                                                                        auto_settings_dict)
                except Exception as _diag_e:
                    import traceback
                    self.msg_if.pub_warn("PTAUTO_DIAG auto_process RAISED: " + repr(_diag_e))
                    self.msg_if.pub_warn("PTAUTO_DIAG traceback:\n" + traceback.format_exc())
            # Only absorb mode flags the CONTROLLER changed relative to what we fed
            # it this cycle (e.g. the Sweep->Stab fallback). If the controller left
            # a flag unchanged, do NOT write it back -- self.* may have just been
            # updated by a concurrent RUI toggle callback, and echoing the stale
            # input value would clobber that fresh user change. That stale-echo
            # write-back was the "toggle bounces off then back on" race.
            if auto_data_dict['scan_enabled'] != _in_scan_enabled:
                self.scan_enabled = auto_data_dict['scan_enabled']
                if self.node_if is not None:
                    self.node_if.set_param('scan_enabled', self.scan_enabled)
            self.scan_pan_enabled = self.scan_enabled
            self.scan_tilt_enabled = self.scan_enabled

            
            if self.track_ready == True:
                if auto_data_dict['track_enabled'] != _in_track_enabled:
                    self.track_enabled = auto_data_dict['track_enabled']
                    if self.node_if is not None:
                        self.node_if.set_param('track_enabled', self.track_enabled)
                self.track_pan_enabled = self.track_enabled
                self.track_tilt_enabled = self.track_enabled

            if self.scan_ready == True:
                if auto_data_dict['stab_enabled'] != _in_stab_enabled:
                    self.stab_enabled = auto_data_dict['stab_enabled']
                    if self.node_if is not None:
                        self.node_if.set_param('stab_enabled', self.stab_enabled)
                self.stab_pan_enabled = self.stab_enabled
                self.stab_tilt_enabled = self.stab_enabled
                

            best_target_dict = auto_data_dict['target_dict'] 
            if best_target_dict is not None:
                self.track_last_time = nepi_utils.get_time()
                self.track_dict = best_target_dict
                self.track_dict_check = best_target_dict
                ##################
                track_msg = nepi_targets.convert_target_dict2msg(best_target_dict)
                # if self.node_if is not None:
                #     self.node_if.publish_pub('ai_track', track_msg) 




           
            auto_is_night = auto_data_dict['auto_is_night'] 
            self.is_night = auto_is_night
            auto_night_updated = auto_data_dict['auto_night_updated'] 
            if auto_night_updated == True:
                if self.auto_night_mode_enabled == True:
                    self.night_mode_enabled = auto_is_night
                    self.disables_need_update = True
            auto_data_dict['auto_night_updated'] = False

            

    ##########################
    # Update stab settings and data dictionaries                                                                    
    #self.msg_if.pub_warn("Stabs update process complete", throttle_s=1)           
    auto_data_dict['last_auto_time'] = nepi_utils.get_time()
    self.tracking_dict = auto_data_dict['tracking_dict']
    self.auto_processes_dict[selected_auto_process] = auto_settings_dict
    self.auto_dict_lock.acquire()
    self.auto_data_dict = auto_data_dict
    self.auto_dict_lock.release()
    # Keep the produced dict as next cycle's `last` (reference-swap, no deepcopy).
    self._auto_data_dict_prev = auto_data_dict


    
  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    self.msg_if.pub_info(" Shutting down: Executing script cleanup actions")
    self._auto_loop_stop = True
    t = getattr(self, '_auto_thread', None)
    if t is not None:
        t.join(timeout=2.0)




#########################################
# Main
#########################################
if __name__ == '__main__':
  NepiPanTiltAutoApp()
