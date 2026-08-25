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
# Redistributions in source code must retain this top-level comment block.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com
#

import copy
import threading

from std_msgs.msg import Empty, String, Bool, Float32

from nepi_interfaces.msg import ControlsStatus
from nepi_interfaces.msg import DevicePTXStatus
from nepi_interfaces.msg import ImageStatus
from nepi_interfaces.msg import NavPoseStatus
from nepi_interfaces.msg import ProcessStatus
from nepi_interfaces.msg import TargetingStatus
from nepi_interfaces.msg import Track
from nepi_interfaces.msg import FloatArray, StringArray

from nepi_app_auto_turret.msg import AutoTurretStatus

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_track

from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF
from nepi_api.system_if import SaveDataIF
from nepi_api.connect_device_if_ptx import ConnectPTXDeviceIF
from nepi_api.connect_data_if import ConnectImageIF
from nepi_api.connect_data_if import ConnectNavPoseIF
from nepi_api.connect_targets_if import ConnectTargetsIF


#########################################
# Node Class
#########################################

# Documented unset sentinel for every numeric field in AutoTurretStatus.
UNSET_VALUE = -999

# The image publisher node's watchdog shuts it down after 3 s without a parent
# status message (WATCHDOG_TIMEOUT in auto_turret_app_img_pub_node.py), so the
# status cadence is not a display preference -- 1 Hz is the contract.
STATUS_PUBLISH_RATE_HZ = 1.0
UPDATER_RATE_HZ = 1.0

IMG_PUB_NODE_SUFFIX = '_img_pub'
IMG_PUB_NODE_FILE = 'auto_turret_app_img_pub_node.py'
IMG_PUB_DATA_PRODUCT = 'process_image'

PKG_NAME = 'nepi_app_auto_turret'

# Track.target is a Target msg, but ConnectTargetsIF delivers targets as dicts,
# so the selected one is rebuilt through nepi_sdk.convert_dict2msg.
TARGET_MSG_TYPE = 'nepi_interfaces/Target'

PROCESS_NAME = 'auto_turret'
PROCESS_GROUP = 'AUTOMATION'
PROCESS_DESCRIPTION = 'Pan tilt turret automation process'


class NepiAutoTurretApp(object):

  #######################
  ### Node Initialization

  DEFAULT_NODE_NAME = "app_auto_turret"  # Can be overwritten by launch command

  node_if = None
  save_data_if = None

  data_products = [IMG_PUB_DATA_PRODUCT]

  # Source connections. Each Connect*IF owns one selector row end to end:
  # discovery into available_topics, selected_topic with its own persisted param,
  # a select_topic subscriber, check_connection(), and a ConnectIFStatus published
  # on <node>/<connect_name>. The matching RUI components (Nepi_IF_ConnectPTX,
  # Nepi_IF_ConnectIDX, Nepi_IF_ConnectTargets, Nepi_IF_ConnectNavPose) render each
  # row off that status, so this node neither discovers nor selects sources itself.
  pantilt_connect_if = None
  image_connect_if = None
  targets_connect_if = None
  navpose_connect_if = None

  pantilt_connected = False
  image_connected = False
  targets_connected = False
  navpose_connected = False

  # Pan tilt state
  pt_position = None
  pan_goto = UNSET_VALUE
  tilt_goto = UNSET_VALUE
  speed_ratio = 0.5
  pan_speed_ratio = 0.5
  tilt_speed_ratio = 0.5
  last_speed_ratios_pushed = None

  # Auto modes. No control loop drives these yet; see setScanningEnableCb.
  scanning_enabled = False
  tracking_enabled = False
  stabilize_enabled = False

  # Process controls
  auto_select_enabled = True
  max_process_rate_hz = 10.0
  max_image_pub_rate_hz = 10.0

  # Overlay controls, consumed by the image publisher node off the status msg
  show_full_screen = False
  show_targets_enabled = False
  show_track_enabled = False
  show_crosshair_enabled = False
  crosshair_offset_degs = [0.0, 0.0]

  # Child overlay image publisher node
  img_pub_sub_process = None
  img_pub_node_name = None
  img_pub_topic = 'None'

  tracking_dict = None
  status_lock = threading.Lock()

  def __init__(self):
    #### APP NODE INIT SETUP ####
    nepi_sdk.init_node(name = self.DEFAULT_NODE_NAME)
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
    self.tracking_dict = copy.deepcopy(nepi_track.BLANK_SETTINGS_DICT)

    # Every connector is seeded here, before node_if setup, because
    # initCb(do_updates = True) below publishes a status during construction --
    # get_status_msg() reads all four connectors and would raise on a missing
    # attribute long before the connectors themselves are built at the bottom of
    # this method. The class attributes above already cover a connector whose
    # constructor raises; these assignments make the ordering contract explicit.
    self.pantilt_connect_if = None
    self.image_connect_if = None
    self.targets_connect_if = None
    self.navpose_connect_if = None

    # The image publisher node recovers this namespace by stripping the suffix
    # from its own, so the child node name is load-bearing, not cosmetic.
    self.img_pub_node_name = self.node_name + IMG_PUB_NODE_SUFFIX
    self.img_pub_topic = nepi_sdk.create_namespace(
                            self.node_namespace,
                            IMG_PUB_DATA_PRODUCT)

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
    # Param wire names ARE namespace + key, so these keys are part of the
    # external surface and cannot be renamed freely.
    # Source selection is NOT persisted here. Each Connect*IF registers and
    # restores its own selected_topic param under its own connect namespace.
    self.PARAMS_DICT = {
        'scanning_enabled': {
            'namespace': self.node_namespace,
            'factory_val': self.scanning_enabled
        },
        'tracking_enabled': {
            'namespace': self.node_namespace,
            'factory_val': self.tracking_enabled
        },
        'stabilize_enabled': {
            'namespace': self.node_namespace,
            'factory_val': self.stabilize_enabled
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
        'auto_select_enabled': {
            'namespace': self.node_namespace,
            'factory_val': self.auto_select_enabled
        },
        'max_process_rate_hz': {
            'namespace': self.node_namespace,
            'factory_val': self.max_process_rate_hz
        },
        'max_image_pub_rate_hz': {
            'namespace': self.node_namespace,
            'factory_val': self.max_image_pub_rate_hz
        },
        'show_full_screen': {
            'namespace': self.node_namespace,
            'factory_val': self.show_full_screen
        },
        'show_targets_enabled': {
            'namespace': self.node_namespace,
            'factory_val': self.show_targets_enabled
        },
        'show_track_enabled': {
            'namespace': self.node_namespace,
            'factory_val': self.show_track_enabled
        },
        'show_crosshair_enabled': {
            'namespace': self.node_namespace,
            'factory_val': self.show_crosshair_enabled
        },
        'crosshair_offset_degs': {
            'namespace': self.node_namespace,
            'factory_val': self.crosshair_offset_degs
        }
    }

    # Publishers Config Dict ####################
    self.PUBS_DICT = {
        'status_pub': {
            'namespace': self.node_namespace,
            'topic': 'status',
            'msg': AutoTurretStatus,
            'qsize': 1,
            'latch': True
        },
        'track_pub': {
            'namespace': self.node_namespace,
            'topic': 'track',
            'msg': Track,
            'qsize': 1,
            'latch': False
        }
    }

    # Subscribers Config Dict ####################
    # The full command surface registers here, once, unconditionally. Every
    # callback guards on its own preconditions instead, so a missing pan tilt
    # device or an unimplemented mode never changes which topics exist.
    self.SUBS_DICT = {
        #####################
        ### Generic process source selection.
        #####################
        # Source selection proper belongs to the three Connect*IFs, each of which
        # registers its own select_topic under its own connect namespace. These
        # generic aliases stay because process_status advertises
        # available_source_topics; they forward to the IDX connector, which is
        # this app's one data source.
        'add_source_topic': {
            'namespace': self.node_namespace,
            'topic': 'add_source_topic',
            'msg': String,
            'qsize': 1,
            'callback': self.addSourceTopicCb,
            'callback_args': ()
        },
        'remove_source_topic': {
            'namespace': self.node_namespace,
            'topic': 'remove_source_topic',
            'msg': String,
            'qsize': 1,
            'callback': self.removeSourceTopicCb,
            'callback_args': ()
        },
        'add_source_topics': {
            'namespace': self.node_namespace,
            'topic': 'add_source_topics',
            'msg': StringArray,
            'qsize': 1,
            'callback': self.addSourceTopicsCb,
            'callback_args': ()
        },
        'remove_source_topics': {
            'namespace': self.node_namespace,
            'topic': 'remove_source_topics',
            'msg': StringArray,
            'qsize': 1,
            'callback': self.removeSourceTopicsCb,
            'callback_args': ()
        },
        'set_auto_select_enable': {
            'namespace': self.node_namespace,
            'topic': 'set_auto_select_enable',
            'msg': Bool,
            'qsize': 1,
            'callback': self.setAutoSelectEnableCb,
            'callback_args': ()
        },
        'set_max_process_rate': {
            'namespace': self.node_namespace,
            'topic': 'set_max_process_rate',
            'msg': Float32,
            'qsize': 1,
            'callback': self.setMaxProcessRateCb,
            'callback_args': ()
        },
        'set_max_image_pub_rate': {
            'namespace': self.node_namespace,
            'topic': 'set_max_image_pub_rate',
            'msg': Float32,
            'qsize': 1,
            'callback': self.setMaxImagePubRateCb,
            'callback_args': ()
        },

        #####################
        ### Pan tilt controls
        #####################
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
        'pt_stop': {
            'namespace': self.node_namespace,
            'topic': 'pt_stop',
            'msg': Empty,
            'qsize': 1,
            'callback': self.ptStopCb,
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
        ### Auto modes
        #####################
        'set_scanning_enable': {
            'namespace': self.node_namespace,
            'topic': 'set_scanning_enable',
            'msg': Bool,
            'qsize': 1,
            'callback': self.setScanningEnableCb,
            'callback_args': ()
        },
        'set_tracking_enable': {
            'namespace': self.node_namespace,
            'topic': 'set_tracking_enable',
            'msg': Bool,
            'qsize': 1,
            'callback': self.setTrackingEnableCb,
            'callback_args': ()
        },
        'set_stabilize_enable': {
            'namespace': self.node_namespace,
            'topic': 'set_stabilize_enable',
            'msg': Bool,
            'qsize': 1,
            'callback': self.setStabilizeEnableCb,
            'callback_args': ()
        },

        #####################
        ### Overlay controls
        #####################
        'set_full_screen': {
            'namespace': self.node_namespace,
            'topic': 'set_full_screen',
            'msg': Bool,
            'qsize': 1,
            'callback': self.setFullScreenCb,
            'callback_args': ()
        },
        'set_show_targets': {
            'namespace': self.node_namespace,
            'topic': 'set_show_targets',
            'msg': Bool,
            'qsize': 1,
            'callback': self.setShowTargetsCb,
            'callback_args': ()
        },
        'set_show_track': {
            'namespace': self.node_namespace,
            'topic': 'set_show_track',
            'msg': Bool,
            'qsize': 1,
            'callback': self.setShowTrackCb,
            'callback_args': ()
        },
        'set_show_crosshair': {
            'namespace': self.node_namespace,
            'topic': 'set_show_crosshair',
            'msg': Bool,
            'qsize': 1,
            'callback': self.setShowCrosshairCb,
            'callback_args': ()
        },
        'set_crosshair_offset': {
            'namespace': self.node_namespace,
            'topic': 'set_crosshair_offset',
            'msg': FloatArray,
            'qsize': 1,
            'callback': self.setCrosshairOffsetCb,
            'callback_args': ()
        }
    }

    # Create Node Class ####################
    self.node_if = NodeClassIF(
                    configs_dict = self.CFGS_DICT,
                    params_dict = self.PARAMS_DICT,
                    pubs_dict = self.PUBS_DICT,
                    subs_dict = self.SUBS_DICT,
                    msg_if = self.msg_if
                    )
    self.node_if.wait_for_ready()

    ###############################
    # Create System IFs

    # SaveDataIF lands on <node namespace>/save_data, which is what this node
    # reports as process_status.save_data_topic and what the child image
    # publisher node attaches its own pub_status=False instance to. Its own
    # node_if keeps its registry keys clear of this node's.
    factory_data_rates = {}
    for d in self.data_products:
      factory_data_rates[d] = [1.0, 0.0, 100]

    self.save_data_if = SaveDataIF(namespace = self.node_namespace,
                    data_products = self.data_products,
                    pub_status = True,
                    factory_rate_dict = factory_data_rates,
                    msg_if = self.msg_if,
                    # node_if = self.node_if
                    )

    ###############################
    # Create Source Connect IFs
    #
    # Constructed once, unconditionally, with no namespace argument -- each one
    # discovers its own candidates and restores its own persisted selection.
    # Each gets its own node_if (the default) rather than sharing this node's, so
    # their registry keys cannot collide with this node's or with each other's.
    self.pantilt_connect_if = ConnectPTXDeviceIF(
                                    auto_select_enabled = self.auto_select_enabled,
                                    panTiltCb = self.panTiltCb,
                                    stopPanCb = self.stopPanCb,
                                    stopTiltCb = self.stopTiltCb,
                                    show_selector = True,
                                    show_controls = False,
                                    show_data = False,
                                    msg_if = self.msg_if,
                                    # node_if = self.node_if
                                    )

    self.image_connect_if = ConnectImageIF(
                                    connect_name = 'image_connect',
                                    auto_select_enabled = self.auto_select_enabled,
                                    filter_topic_list = ['color_image'],
                                    connect_data = False,
                                    show_selector = True,
                                    show_controls = False,
                                    show_data = False,
                                    msg_if = self.msg_if,
                                    # node_if = self.node_if
                                    )

    self.targets_connect_if = ConnectTargetsIF(
                                    auto_select_enabled = self.auto_select_enabled,
                                    dataCB = self.targetsDataCb,
                                    show_selector = True,
                                    show_controls = False,
                                    show_data = False,
                                    msg_if = self.msg_if,
                                    # node_if = self.node_if
                                    )

    # connect_data = False for the same reason as the image connector: this node
    # reports the selected navpose source and its status, and nothing here
    # consumes NavPose messages, so there is no reason to subscribe to the data
    # topic. The status subscriber is created either way, which is what
    # check_connection() and get_status_msg() report from.
    self.navpose_connect_if = ConnectNavPoseIF(
                                    connect_name = 'navpose_connect',
                                    auto_select_enabled = self.auto_select_enabled,
                                    connect_data = False,
                                    show_selector = True,
                                    show_controls = False,
                                    show_data = False,
                                    msg_if = self.msg_if,
                                    # node_if = self.node_if
                                    )

    for name, connect_if in [('pan tilt', self.pantilt_connect_if),
                              ('image', self.image_connect_if),
                              ('targets', self.targets_connect_if),
                              ('navpose', self.navpose_connect_if)]:
      if connect_if.wait_for_ready(timeout = 10) != True:
        self.msg_if.pub_warn("Connect IF did not become ready: " + str(name))

    ##############################
    self.initCb(do_updates = True)

    ##############################
    # Launch the overlay image publisher node
    self.launchImgPubNode()

    ##########################
    # Complete Initialization
    nepi_sdk.sleep(1)
    nepi_sdk.start_timer_process(float(1) / UPDATER_RATE_HZ, self.updaterCb, oneshot = True)
    nepi_sdk.start_timer_process(float(1) / STATUS_PUBLISH_RATE_HZ, self.statusPublishCb)
    nepi_sdk.on_shutdown(self.shutdownCb)

    #########################################################
    ## Initiation Complete
    self.msg_if.pub_info("Initialization Complete")
    # Spin forever
    nepi_sdk.spin()
    #########################################################

  ###############################
  # Child Node Management
  ###############################

  def launchImgPubNode(self):
    if self.img_pub_sub_process is not None:
      return True
    [success, msg, sub_process] = nepi_sdk.launch_node(
                                    pkg_name = PKG_NAME,
                                    file_name = IMG_PUB_NODE_FILE,
                                    ros_node_name = self.img_pub_node_name,
                                    namespace = self.base_namespace)
    if success == True:
      self.img_pub_sub_process = sub_process
      self.msg_if.pub_info("Launched overlay image publisher node: " + str(self.img_pub_node_name))
    else:
      self.msg_if.pub_warn("Failed to launch overlay image publisher node: " + str(msg))
    return success

  def killImgPubNode(self):
    if self.img_pub_sub_process is None:
      return False
    try:
      nepi_sdk.kill_node_process(self.img_pub_node_name, self.img_pub_sub_process)
    except Exception as e:
      self.msg_if.pub_warn("Failed to stop overlay image publisher node: " + str(e))
    self.img_pub_sub_process = None
    return True

  ###############################
  # Source Connection State
  ###############################

  def updaterCb(self, timer):
    # The connectors run their own discovery, selection and connection loops.
    # This updater only samples their connection state for the status message
    # and re-asserts the stored speed ratios when a pan tilt device connects.
    self.pantilt_connected = self.checkConnected(self.pantilt_connect_if)
    self.image_connected = self.checkConnected(self.image_connect_if)
    self.targets_connected = self.checkConnected(self.targets_connect_if)
    self.navpose_connected = self.checkConnected(self.navpose_connect_if)
    self.pushSpeedRatios()
    nepi_sdk.start_timer_process(float(1) / UPDATER_RATE_HZ, self.updaterCb, oneshot = True)

  def checkConnected(self, connect_if):
    if connect_if is None:
      return False
    try:
      return connect_if.check_connection() == True
    except Exception as e:
      self.msg_if.pub_warn("Connection check failed: " + str(e))
      return False

  def getSelectedTopic(self, connect_if):
    if connect_if is None:
      return 'None'
    topic = connect_if.get_selected_topic()
    if topic is None or topic == '':
      return 'None'
    return topic

  def pushSpeedRatios(self):
    # A newly connected device knows nothing of the ratios this node restored
    # from config, so push them once per connection rather than every cycle.
    if self.pantilt_connected == False:
      self.last_speed_ratios_pushed = None
      return
    ratios = [self.speed_ratio, self.pan_speed_ratio, self.tilt_speed_ratio]
    if ratios == self.last_speed_ratios_pushed:
      return
    self.pantilt_connect_if.set_speed_ratio(self.speed_ratio)
    self.pantilt_connect_if.set_pan_speed_ratio(self.pan_speed_ratio)
    self.pantilt_connect_if.set_tilt_speed_ratio(self.tilt_speed_ratio)
    self.last_speed_ratios_pushed = ratios

  def panTiltCb(self, pan_deg, tilt_deg):
    self.pt_position = [pan_deg, tilt_deg]

  def stopPanCb(self):
    self.pan_goto = UNSET_VALUE

  def stopTiltCb(self):
    self.tilt_goto = UNSET_VALUE

  def targetsDataCb(self, data_dict):
    # ConnectTargetsIF hands over an already-converted dict, not the Targets msg
    # (connect_targets_if.py:434). That suits get_best_from_targets, which works
    # on dicts anyway; only the winner is rebuilt into a Target msg for Track.
    #
    # The child image publisher node draws its own target boxes straight off
    # <image topic>/targets. What it cannot do is pick the tracked one, so that
    # selection happens here and goes out on <node>/track.
    if self.tracking_enabled == False:
      return
    if self.node_if is None:
      return
    data = data_dict.get('data', None)
    if data is None:
      return
    targets_dict_list = data.get('targets', [])
    if len(targets_dict_list) == 0:
      return
    [best_target, tracking_dict] = nepi_track.get_best_from_targets(
                                        targets_dict_list,
                                        tracking_dict = self.tracking_dict)
    self.tracking_dict = tracking_dict
    if best_target is None:
      return
    try:
      target_msg = nepi_sdk.convert_dict2msg(TARGET_MSG_TYPE, best_target)
    except Exception as e:
      self.msg_if.pub_warn("Failed to rebuild target message: " + str(e))
      return
    if target_msg is None:
      return
    track_msg = Track()
    track_msg.timestamp = nepi_utils.get_time()
    track_msg.process_name = PROCESS_NAME
    track_msg.process_namespace = self.node_namespace
    track_msg.source_topic = data.get('source_topic', '')
    track_msg.source_timestamp = data.get('source_timestamp', 0.0)
    track_msg.target = target_msg
    self.node_if.publish_pub('track_pub', track_msg)

  ###############################
  # Source Selection Callbacks
  ###############################

  # The dedicated selectors live on the connectors, at <node>/idx_connect and
  # friends. Everything below is the generic process-source alias surface, which
  # forwards to the IDX connector so both paths cannot disagree about what is
  # selected. set_selected_topic() rejects a topic that is not in the
  # connector's own available list, so no membership check is needed here.

  def setImageTopic(self, topic):
    if self.image_connect_if is None:
      return
    self.msg_if.pub_info("Setting image topic to: " + str(topic))
    self.image_connect_if.set_selected_topic(topic)
    self.publish_status()

  def addSourceTopicCb(self, msg):
    self.setImageTopic(msg.data)

  def removeSourceTopicCb(self, msg):
    if msg.data == self.getSelectedTopic(self.image_connect_if):
      self.setImageTopic('None')

  def addSourceTopicsCb(self, msg):
    # This app consumes one image at a time, so an array set takes the first
    # entry the connector knows about rather than silently dropping the msg.
    available = []
    if self.image_connect_if is not None:
      available = self.image_connect_if.get_available_topics()
    for topic in msg.array:
      if topic in available:
        self.setImageTopic(topic)
        return
    self.msg_if.pub_warn("No known image topic in source topics set: " + str(list(msg.array)))

  def removeSourceTopicsCb(self, msg):
    if self.getSelectedTopic(self.image_connect_if) in msg.array:
      self.setImageTopic('None')

  def setAutoSelectEnableCb(self, msg):
    enabled = msg.data
    self.msg_if.pub_info("Setting auto source select to: " + str(enabled))
    self.auto_select_enabled = enabled
    self.setParam('auto_select_enabled', enabled)
    self.publish_status()

  def setMaxProcessRateCb(self, msg):
    # Stored and reported only. This node runs no per-frame process loop, so
    # nothing consumes the value yet.
    rate = msg.data
    if rate < 0.1:
      self.msg_if.pub_warn("Ignoring out of range max process rate: " + str(rate))
      return
    self.msg_if.pub_info("Setting max process rate to: " + str(rate))
    self.max_process_rate_hz = rate
    self.setParam('max_process_rate_hz', rate)
    self.publish_status()

  def setMaxImagePubRateCb(self, msg):
    rate = msg.data
    if rate < 0.1:
      self.msg_if.pub_warn("Ignoring out of range max image pub rate: " + str(rate))
      return
    self.msg_if.pub_info("Setting max image pub rate to: " + str(rate))
    self.max_image_pub_rate_hz = rate
    self.setParam('max_image_pub_rate_hz', rate)
    self.publish_status()

  ###############################
  # Pan Tilt Control Callbacks
  ###############################

  def setPanPosDegCb(self, msg):
    self.setPanPosDeg(msg.data)

  def setPanPosDeg(self, pos_deg):
    if self.pantilt_connected == False:
      self.msg_if.pub_warn("No pan tilt device connected; ignoring pan position command")
      return
    if self.getPanControlDisabled() == True:
      self.msg_if.pub_warn("An auto mode owns the pan axis; ignoring pan position command")
      return
    self.msg_if.pub_info("Sending pan position command: " + str(pos_deg))
    self.pan_goto = pos_deg
    self.pantilt_connect_if.goto_to_pan_position(pos_deg)
    self.publish_status()

  def setTiltPosDegCb(self, msg):
    self.setTiltPosDeg(msg.data)

  def setTiltPosDeg(self, pos_deg):
    if self.pantilt_connected == False:
      self.msg_if.pub_warn("No pan tilt device connected; ignoring tilt position command")
      return
    if self.getTiltControlDisabled() == True:
      self.msg_if.pub_warn("An auto mode owns the tilt axis; ignoring tilt position command")
      return
    self.msg_if.pub_info("Sending tilt position command: " + str(pos_deg))
    self.tilt_goto = pos_deg
    self.pantilt_connect_if.goto_to_tilt_position(pos_deg)
    self.publish_status()

  def setSpeedRatioCb(self, msg):
    ratio = self.clampRatio(msg.data)
    if ratio is None:
      return
    self.speed_ratio = ratio
    if self.pantilt_connected == True:
      self.pantilt_connect_if.set_speed_ratio(ratio)
    self.setParam('speed_ratio', ratio)
    self.publish_status()

  def setPanSpeedRatioCb(self, msg):
    ratio = self.clampRatio(msg.data)
    if ratio is None:
      return
    self.pan_speed_ratio = ratio
    if self.pantilt_connected == True:
      self.pantilt_connect_if.set_pan_speed_ratio(ratio)
    self.setParam('pan_speed_ratio', ratio)
    self.publish_status()

  def setTiltSpeedRatioCb(self, msg):
    ratio = self.clampRatio(msg.data)
    if ratio is None:
      return
    self.tilt_speed_ratio = ratio
    if self.pantilt_connected == True:
      self.pantilt_connect_if.set_tilt_speed_ratio(ratio)
    self.setParam('tilt_speed_ratio', ratio)
    self.publish_status()

  def clampRatio(self, ratio):
    if ratio < 0.0 or ratio > 1.0:
      self.msg_if.pub_warn("Ignoring out of range ratio: " + str(ratio))
      return None
    return ratio

  def ptStopCb(self, msg):
    if self.pantilt_connected == False:
      self.msg_if.pub_warn("No pan tilt device connected; ignoring stop command")
      return
    self.msg_if.pub_info("Stopping pan tilt motion")
    self.pantilt_connect_if.stop_moving()
    self.pan_goto = UNSET_VALUE
    self.tilt_goto = UNSET_VALUE
    self.publish_status()

  def panHomeCb(self, msg):
    self.goHome('pan')

  def tiltHomeCb(self, msg):
    self.goHome('tilt')

  def goHome(self, axis):
    if self.pantilt_connected == False:
      self.msg_if.pub_warn("No pan tilt device connected; ignoring " + str(axis) + " home command")
      return
    status_msg = self.pantilt_connect_if.get_status_msg()
    if status_msg is None or status_msg.has_homing == False:
      self.msg_if.pub_warn("Connected pan tilt device does not support homing")
      return
    if axis == 'pan':
      self.msg_if.pub_info("Sending pan home command")
      self.pan_goto = status_msg.pan_home_pos_deg
      self.pantilt_connect_if.goto_to_pan_position(status_msg.pan_home_pos_deg)
    else:
      self.msg_if.pub_info("Sending tilt home command")
      self.tilt_goto = status_msg.tilt_home_pos_deg
      self.pantilt_connect_if.goto_to_tilt_position(status_msg.tilt_home_pos_deg)
    self.publish_status()

  ###############################
  # Auto Mode Callbacks
  ###############################

  # The three enables below store and report only. The controller they would
  # drive lives in nepi_auto_pt.pt_auto_2, but nothing in this node builds the
  # auto_data_dict it needs (navpose feed, mount transforms, per-axis servo
  # state, the control loop), so an enabled mode moves no axis. The toggle
  # still round-trips and still gates pan_control_disabled / tilt_control_disabled,
  # which is what the RUI reads.

  def setScanningEnableCb(self, msg):
    enabled = msg.data
    if enabled == True and self.getScanningReady() == False:
      self.msg_if.pub_warn("Scanning not ready; ignoring enable")
      return
    self.msg_if.pub_info("Setting scanning enable to: " + str(enabled))
    self.scanning_enabled = enabled
    self.setParam('scanning_enabled', enabled)
    self.publish_status()

  def setTrackingEnableCb(self, msg):
    enabled = msg.data
    if enabled == True and self.getTrackingReady() == False:
      self.msg_if.pub_warn("Tracking not ready; ignoring enable")
      return
    self.msg_if.pub_info("Setting tracking enable to: " + str(enabled))
    self.tracking_enabled = enabled
    self.setParam('tracking_enabled', enabled)
    self.publish_status()

  def setStabilizeEnableCb(self, msg):
    enabled = msg.data
    if enabled == True and self.getStabilizeReady() == False:
      self.msg_if.pub_warn("Stabilize not ready; ignoring enable")
      return
    self.msg_if.pub_info("Setting stabilize enable to: " + str(enabled))
    self.stabilize_enabled = enabled
    self.setParam('stabilize_enabled', enabled)
    self.publish_status()

  ###############################
  # Overlay Control Callbacks
  ###############################

  def setFullScreenCb(self, msg):
    # Display state only. No node consumes it; the RUI renders from it.
    enabled = msg.data
    self.msg_if.pub_info("Setting full screen to: " + str(enabled))
    self.show_full_screen = enabled
    self.setParam('show_full_screen', enabled)
    self.publish_status()

  def setShowTargetsCb(self, msg):
    enabled = msg.data
    self.msg_if.pub_info("Setting show targets to: " + str(enabled))
    self.show_targets_enabled = enabled
    self.setParam('show_targets_enabled', enabled)
    self.publish_status()

  def setShowTrackCb(self, msg):
    enabled = msg.data
    self.msg_if.pub_info("Setting show track to: " + str(enabled))
    self.show_track_enabled = enabled
    self.setParam('show_track_enabled', enabled)
    self.publish_status()

  def setShowCrosshairCb(self, msg):
    enabled = msg.data
    self.msg_if.pub_info("Setting show crosshair to: " + str(enabled))
    self.show_crosshair_enabled = enabled
    self.setParam('show_crosshair_enabled', enabled)
    self.publish_status()

  def setCrosshairOffsetCb(self, msg):
    # The image publisher node unpacks exactly two entries.
    offsets = list(msg.array)
    if len(offsets) != 2:
      self.msg_if.pub_warn("Crosshair offset needs two entries, got: " + str(offsets))
      return
    self.msg_if.pub_info("Setting crosshair offset to: " + str(offsets))
    self.crosshair_offset_degs = offsets
    self.setParam('crosshair_offset_degs', offsets)
    self.publish_status()

  ###############################
  # Derived State
  ###############################

  # Connection state is sampled from each connector's own check_connection() in
  # updaterCb rather than inferred from status-message staleness here.

  def getImageConnected(self):
    return self.image_connected

  def getTargeterConnected(self):
    return self.targets_connected

  def getNavPoseTopic(self):
    if self.pantilt_connect_if is None:
      return ''
    status_msg = self.pantilt_connect_if.get_status_msg()
    if status_msg is None:
      return ''
    return status_msg.navpose_topic

  def getScanningReady(self):
    if self.pantilt_connected == False or self.pantilt_connect_if is None:
      return False
    status_msg = self.pantilt_connect_if.get_status_msg()
    if status_msg is None:
      return False
    return status_msg.has_adjustable_speed == True

  def getTrackingReady(self):
    return self.pantilt_connected == True and self.getTargeterConnected() == True

  def getStabilizeReady(self):
    navpose_topic = self.getNavPoseTopic()
    if navpose_topic == '':
      return False
    return self.pantilt_connected == True and nepi_sdk.check_for_topic(navpose_topic) == True

  def getPanControlDisabled(self):
    return (self.scanning_enabled == True or
            self.tracking_enabled == True or
            self.stabilize_enabled == True)

  def getTiltControlDisabled(self):
    return self.getPanControlDisabled()

  def setParam(self, param_name, value):
    if self.node_if is None:
      return
    self.node_if.set_param(param_name, value)
    self.node_if.save_config()

  #######################
  ### Config Functions

  def initCb(self, do_updates = False):
    if self.node_if is not None:
      # Selected sources are restored by each connector from its own param.
      self.scanning_enabled = self.node_if.get_param('scanning_enabled')
      self.tracking_enabled = self.node_if.get_param('tracking_enabled')
      self.stabilize_enabled = self.node_if.get_param('stabilize_enabled')
      self.speed_ratio = self.node_if.get_param('speed_ratio')
      self.pan_speed_ratio = self.node_if.get_param('pan_speed_ratio')
      self.tilt_speed_ratio = self.node_if.get_param('tilt_speed_ratio')
      self.auto_select_enabled = self.node_if.get_param('auto_select_enabled')
      self.max_process_rate_hz = self.node_if.get_param('max_process_rate_hz')
      self.max_image_pub_rate_hz = self.node_if.get_param('max_image_pub_rate_hz')
      self.show_full_screen = self.node_if.get_param('show_full_screen')
      self.show_targets_enabled = self.node_if.get_param('show_targets_enabled')
      self.show_track_enabled = self.node_if.get_param('show_track_enabled')
      self.show_crosshair_enabled = self.node_if.get_param('show_crosshair_enabled')
      self.crosshair_offset_degs = list(self.node_if.get_param('crosshair_offset_degs'))
    if do_updates == True:
      pass
    self.publish_status()

  def resetCb(self, do_updates = True):
    self.msg_if.pub_warn("Reseting")
    if self.node_if is not None:
      pass
    if do_updates == True:
      pass
    self.initCb(do_updates = do_updates)

  def factoryResetCb(self, do_updates = True):
    self.msg_if.pub_warn("Factory Reseting")
    if self.node_if is not None:
      pass
    if do_updates == True:
      pass
    self.initCb(do_updates = do_updates)

  ###################
  ## Status Publishers

  def statusPublishCb(self, timer):
    self.publish_status()

  def get_process_status_msg(self):
    """Build the embedded ProcessStatus sub-message.

    The child overlay image publisher node reads enabled, msg_str,
    image_pub_enabled, max_image_pub_rate_hz and use_last_image off this
    sub-message, and the RUI reads its source-selection and stats fields, so
    every field is assigned here rather than left at its default.

    Returns:
        nepi_interfaces/ProcessStatus: the fully populated sub-message.
    """
    image_connected = self.getImageConnected()
    selected_image_topic = self.getSelectedTopic(self.image_connect_if)
    available_image_topics = []
    if self.image_connect_if is not None:
      available_image_topics = list(self.image_connect_if.get_available_topics())
    selected_sources = []
    if selected_image_topic != 'None':
      selected_sources = [selected_image_topic]

    msg = ProcessStatus()
    msg.name = PROCESS_NAME
    msg.group = PROCESS_GROUP
    msg.description = PROCESS_DESCRIPTION

    msg.node_name = self.node_name
    msg.namespace = self.node_namespace

    msg.data_products = self.data_products
    msg.save_data_topic = nepi_sdk.create_namespace(self.node_namespace, 'save_data')

    msg.enabled = True
    msg.running = True
    msg.state = image_connected
    if image_connected == True:
      msg.msg_str = ProcessStatus.STATE_PROCESSING
    elif selected_image_topic != 'None':
      msg.msg_str = ProcessStatus.STATE_WAITING
    else:
      msg.msg_str = ProcessStatus.STATE_LISTENING

    msg.max_process_rate_hz = self.max_process_rate_hz
    msg.available_processes = []
    msg.selected_process = ''
    # This app exposes no ControlsIF, so the controls report is empty rather
    # than absent. See the RUI note in the session report.
    msg.process_controls = ControlsStatus()

    msg.multi_source_enabled = False
    msg.auto_select_enabled = self.auto_select_enabled
    msg.auto_select_active = self.auto_select_enabled
    msg.available_source_topics = available_image_topics
    msg.selected_sources = selected_sources
    msg.sources_connected = [image_connected] * len(selected_sources)
    msg.sources_pub_namespaces = []

    msg.source_selected = (selected_image_topic != 'None')
    msg.source_connected = image_connected

    msg.has_image_pub = True
    msg.image_pub_name = IMG_PUB_DATA_PRODUCT
    msg.image_pub_enabled = (selected_image_topic != 'None')
    msg.max_image_pub_rate_hz = self.max_image_pub_rate_hz
    msg.use_last_image = True

    msg.imaging_source_topics = selected_sources
    # string[] in ProcessStatus.msg. Assigning the bare string made genpy
    # serialize it one character per element, so the RUI source selector got a
    # list of single characters, none of which matched a live topic.
    msg.imaging_pub_topics = [self.img_pub_topic]

    msg.avg_source_latency = UNSET_VALUE
    msg.avg_source_rate = UNSET_VALUE

    msg.avg_preprocess_latency = UNSET_VALUE
    msg.avg_preprocess_rate = UNSET_VALUE

    msg.avg_process_latency = UNSET_VALUE
    msg.avg_process_rate = UNSET_VALUE

    msg.max_process_rate = self.max_process_rate_hz

    msg.show_selector = True
    msg.show_controls = True
    msg.show_data = True
    msg.show_results = True

    return msg

  def get_status_msg(self):
    """Build the AutoTurretStatus message from current node state.

    Every field of the message is assigned on every call. Numeric fields with
    no live source report the documented unset sentinel of -999 rather than a
    fabricated value.

    Returns:
        nepi_app_auto_turret/AutoTurretStatus: the fully populated status message.
    """



    status_msg = AutoTurretStatus()

    status_msg.process_status = self.get_process_status_msg()


    # Each connect IF is None until its source is selected, so the local has to
    # be seeded before the guard and the empty fallback has to be the field's
    # own type -- a DevicePTXStatus in the image/targets/navpose slots fails
    # serialization on publish. The connected flags come from the cached
    # self.*_connected sampled in updaterCb, not from a check_connection() call
    # on the status path.
    pantilt_status_msg = None
    if self.pantilt_connect_if is not None:
      self.pantilt_connect_if.set_auto_connect_enable(self.auto_select_enabled)
      status_msg.selected_pantilt_topic = self.pantilt_connect_if.get_namespace()
      pantilt_status_msg = self.pantilt_connect_if.get_status_msg()
    if pantilt_status_msg is None:
      status_msg.selected_pantilt_topic = "None"
      pantilt_status_msg = DevicePTXStatus()
    status_msg.pantilt_connected = self.pantilt_connected
    status_msg.pantilt_status_msg = pantilt_status_msg

    image_status_msg = None
    if self.image_connect_if is not None:
      self.image_connect_if.set_auto_connect_enable(self.auto_select_enabled)
      status_msg.selected_image_topic = self.image_connect_if.get_namespace()
      image_status_msg = self.image_connect_if.get_status_msg()
    if image_status_msg is None:
      status_msg.selected_image_topic = "None"
      image_status_msg = ImageStatus()
    status_msg.image_connected = self.image_connected
    status_msg.image_status_msg = image_status_msg

    targets_status_msg = None
    if self.targets_connect_if is not None:
      self.targets_connect_if.set_auto_connect_enable(self.auto_select_enabled)
      status_msg.selected_targets_topic = self.targets_connect_if.get_namespace()
      targets_status_msg = self.targets_connect_if.get_status_msg()
    if targets_status_msg is None:
      status_msg.selected_targets_topic = "None"
      targets_status_msg = TargetingStatus()
    status_msg.targets_connected = self.targets_connected
    status_msg.targets_status_msg = targets_status_msg

    navpose_status_msg = None
    if self.navpose_connect_if is not None:
      self.navpose_connect_if.set_auto_connect_enable(self.auto_select_enabled)
      status_msg.selected_navpose_topic = self.navpose_connect_if.get_namespace()
      navpose_status_msg = self.navpose_connect_if.get_status_msg()
    if navpose_status_msg is None:
      status_msg.selected_navpose_topic = "None"
      navpose_status_msg = NavPoseStatus()
    status_msg.navpose_connected = self.navpose_connected
    status_msg.navpose_status_msg = navpose_status_msg

    status_msg.scanning_ready = self.getScanningReady()
    status_msg.scanning_enabled = self.scanning_enabled

    status_msg.tracking_ready = self.getTrackingReady()
    status_msg.tracking_enabled = self.tracking_enabled

    status_msg.stabilize_ready = self.getStabilizeReady()
    status_msg.stabilize_enabled = self.stabilize_enabled

    status_msg.pan_control_disabled = self.getPanControlDisabled()
    status_msg.tilt_control_disabled = self.getTiltControlDisabled()

    if self.pantilt_connect_if is not None:
      status_msg.pan_tilt_max_speed_dps = self.pantilt_connect_if.get_pan_tilt_max_speed_dps()
    else:
      status_msg.pan_tilt_max_speed_dps = UNSET_VALUE

    # Nothing in this node measures pan tilt command-to-motion latency.
    status_msg.pan_tilt_avg_move_delay = UNSET_VALUE

    status_msg.speed_ratio = self.speed_ratio
    status_msg.pan_speed_ratio = self.pan_speed_ratio
    status_msg.tilt_speed_ratio = self.tilt_speed_ratio

    if self.pt_position is not None:
      status_msg.pan_deg = self.pt_position[0]
      status_msg.tilt_deg = self.pt_position[1]
    else:
      status_msg.pan_deg = UNSET_VALUE
      status_msg.tilt_deg = UNSET_VALUE

    if self.pantilt_connected == True:
      status_msg.pan_goal = pantilt_status_msg.pan_goal_deg
      status_msg.tilt_goal = pantilt_status_msg.tilt_goal_deg
      status_msg.pan_deg_per_sec = pantilt_status_msg.speed_pan_dps
      status_msg.tilt_deg_per_sec = pantilt_status_msg.speed_tilt_dps
    else:
      status_msg.pan_goal = UNSET_VALUE
      status_msg.tilt_goal = UNSET_VALUE
      status_msg.pan_deg_per_sec = UNSET_VALUE
      status_msg.tilt_deg_per_sec = UNSET_VALUE

    status_msg.pan_goto = self.pan_goto
    status_msg.tilt_goto = self.tilt_goto

    status_msg.image_pub_topic = self.img_pub_topic

    status_msg.show_full_screen = self.show_full_screen
    status_msg.show_targets_enabled = self.show_targets_enabled
    status_msg.show_track_enabled = self.show_track_enabled
    status_msg.show_crosshair_enabled = self.show_crosshair_enabled
    status_msg.crosshair_offset_degs = self.crosshair_offset_degs

    return status_msg

  def publish_status(self):
    """Publish the current AutoTurretStatus on the node's status topic.

    Called on the status timer and after every accepted command so the RUI and
    the child image publisher node both see a change immediately.
    """
    if self.node_if is None:
      return
    self.status_lock.acquire()
    try:
      status_msg = self.get_status_msg()
      self.node_if.publish_pub('status_pub', status_msg)
    finally:
      self.status_lock.release()

  #######################
  # Node Cleanup

  def shutdownCb(self):
    print("Shutting down: Executing script cleanup actions")
    #nepi_sdk.kill_node(self.img_pub_node_name)


#########################################
# Main
#########################################
if __name__ == '__main__':
  NepiAutoTurretApp()
