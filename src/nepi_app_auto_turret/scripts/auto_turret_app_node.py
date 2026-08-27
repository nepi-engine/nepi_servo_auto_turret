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

from nepi_interfaces.msg import DevicePTXStatus
from nepi_interfaces.msg import ImageStatus
from nepi_interfaces.msg import NavPoseStatus
from nepi_interfaces.msg import TargetingStatus
from nepi_interfaces.msg import Track

from nepi_interfaces.msg import Datum, DataStatus, Control, ControlsStatus

from nepi_interfaces.msg import FloatArray, StringArray

from nepi_app_auto_turret.msg import AutoTurretStatus

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_targets
from nepi_sdk import nepi_targets_track
from nepi_sdk import nepi_controls
from nepi_sdk import nepi_data


from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF
from nepi_api.system_if import SaveDataIF
from nepi_api.process_if import ProcessIF
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

# The four processes this node owns, keyed by the short name used everywhere
# below: the enable param, the status fields, and the ProcessIF instance.
#
# 'auto' is the supervisor. It is the nepi_auto_pt.pt_auto_2 controller, which
# owns the pan/tilt loop; the other three are the mode selectors that controller
# chooses between. That is the controller's own model, not a layer invented
# here -- see the MODE MODEL block in pt_auto_2: STAB beats TRACK beats SCAN
# beats HOLD, and STAB forces SCAN and TRACK off. So scanning and tracking are
# not mutually exclusive with each other (the controller picks TRACK when a
# fresh detection exists and falls back to SCAN when one does not), but
# enabling stabilize clears both.
AUTO_PROCESS = 'auto'
SCAN_PROCESS = 'scan'
TRACK_PROCESS = 'track'
STAB_PROCESS = 'stab'
PROCESS_KEYS = [AUTO_PROCESS, SCAN_PROCESS, TRACK_PROCESS, STAB_PROCESS]

# The mode processes, in the controller's own priority order. Enabling one
# clears every mode listed after it that the controller would force off.
MODE_PROCESS_KEYS = [STAB_PROCESS, TRACK_PROCESS, SCAN_PROCESS]

# Enabling stabilize forces scanning and tracking off, because pt_auto_2 writes
# that back into the data dict anyway. Doing it here keeps the reported state
# from disagreeing with what the controller will do.
PROCESS_CLEARS = {
    STAB_PROCESS: [TRACK_PROCESS, SCAN_PROCESS]
}

# Until the pt_auto_2 loop is wired to a live auto_data_dict, an enabled process
# is enabled and not running, and says so. Reporting running = enabled here
# would put a green Running indicator on a loop that does not exist.
PROCESS_NOT_RUNNING_MSG = 'Enabled. Control loop not yet wired; no axis is driven.'


class NepiAutoTurretApp(object):

  #######################
  ### Node Initialization

  DEFAULT_NODE_NAME = "app_auto_turret"  # Can be overwritten by launch command

  WATCHDOG_TARGETS_TIMEOUT = 1

  node_if = None
  save_data_if = None

  data_products = [IMG_PUB_DATA_PRODUCT]

  status_msg = AutoTurretStatus()

    
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

  pan_control_manaul_enabled = True
  tilt_control_manaul_enabled = True

  pan_control_auto_enabled = False
  tilt_control_auto_enabled = False

  # Auto modes. No control loop drives these yet; see setScanningEnableCb.

  # Process controls
  auto_select_enabled = True
  max_process_rate_hz = 10.0
  max_image_pub_rate_hz = 10.0

  auto_enabled = False
  auto_process_name = 'process_auto'
  auto_process_namespace = ''
  auto_process_if = None
  auto_process_controls = copy.deepcopy(nepi_controls.EXAMPLE_INIT_DICT)
  auto_process_data = copy.deepcopy(nepi_data.EXAMPLE_INIT_DICT)

  scanning_enabled = False
  scan_process_name = 'process_scan'
  scan_process_namespace = ''
  scan_process_if = None
  scan_process_controls = copy.deepcopy(nepi_controls.EXAMPLE_INIT_DICT)
  scan_process_data = copy.deepcopy(nepi_data.EXAMPLE_INIT_DICT)

  tracking_enabled = False
  track_process_name = 'process_track'
  track_process_namespace = ''
  track_process_if = None
  track_process_controls = copy.deepcopy(nepi_controls.EXAMPLE_INIT_DICT)
  track_process_data = copy.deepcopy(nepi_data.EXAMPLE_INIT_DICT)

  stabilize_enabled = False
  stab_process_name = 'process_stab'
  stab_process_namespace = ''
  stab_process_if = None
  stab_process_controls = copy.deepcopy(nepi_controls.EXAMPLE_INIT_DICT)
  stab_process_data = copy.deepcopy(nepi_data.EXAMPLE_INIT_DICT)

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

  last_targets_time = 0
  targets_dict_list = None
  targets_lock = threading.Lock()

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
   
    self.status_msg.node_name = self.node_name
    self.status_msg.namespace = self.node_namespace



    self.auto_process_namespace = self.node_namespace + '/' + self.auto_process_name

    self.scan_process_namespace = self.node_namespace + '/' + self.scan_process_name

    self.track_process_namespace = self.node_namespace + '/' + self.track_process_name
    
    self.stab_process_namespace = self.node_namespace + '/' + self.stab_process_name


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
        'auto_enabled': {
            'namespace': self.node_namespace,
            'factory_val': self.auto_enabled
        },
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
        # Ratio position commands sit beside the degree ones because that is
        # what the image viewer's flanking pan and tilt sliders speak. The
        # sliders publish here, to the app, never to the pan tilt device -- the
        # app owns the gating (connected, and no auto mode holding the axis) and
        # forwards to the device. Ratio also spares the RUI from having to know
        # the connected device's soft stops to build a degree command; the
        # device maps 0.0-1.0 onto its own travel and reports the result back as
        # pan_goal_ratio/tilt_goal_ratio in its status.
        'set_pan_pos_ratio': {
            'namespace': self.node_namespace,
            'topic': 'set_pan_pos_ratio',
            'msg': Float32,
            'qsize': 1,
            'callback': self.setPanPosRatioCb,
            'callback_args': ()
        },
        'set_tilt_pos_ratio': {
            'namespace': self.node_namespace,
            'topic': 'set_tilt_pos_ratio',
            'msg': Float32,
            'qsize': 1,
            'callback': self.setTiltPosRatioCb,
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
        'set_auto_enable': {
            'namespace': self.node_namespace,
            'topic': 'set_auto_enable',
            'msg': Bool,
            'qsize': 1,
            'callback': self.setAutoEnableCb,
            'callback_args': ()
        },
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

    self.status_msg.data_products = self.data_products
    self.status_msg.save_data_topic = nepi_sdk.create_namespace(self.node_namespace, 'save_data')

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
                                    dataCB = self.targetsCb,
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
                                    filter_topic_list=['navposes'],
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
    # One ProcessIF per process. Each gets has_enable = True and its own enable
    # callback, so the process panel's Enable toggle and this node's own
    # set_<mode>_enable topic land on the same setProcessEnable() path and can
    # never report different states. Every callback binds the process key rather
    # than closing over a loop variable.
    self.auto_process_if = ProcessIF(process_name = self.auto_process_name,
                process_group = self.node_name,
                process_description = 'Pan tilt auto control supervisor',
                process_data_dict = self.auto_process_data,
                process_controls_dict = self.auto_process_controls,
                process_status_msg = None,
                show_controls = True,
                show_data = True,
                has_enable = True,
                enabled = self.auto_enabled,
                enableCb = self.autoEnableCb,
                log_name = None,
                log_name_list = [],
                msg_if = self.msg_if,
                node_if = self.node_if
    )

    self.scan_process_if = ProcessIF(process_name = self.scan_process_name,
                process_group = self.node_name,
                process_description = 'Pan tilt scan mode',
                process_data_dict = self.scan_process_data,
                process_controls_dict = self.scan_process_controls,
                process_status_msg = None,
                show_controls = True,
                show_data = True,
                has_enable = True,
                enabled = self.scanning_enabled,
                enableCb = self.scanEnableCb,
                log_name = None,
                log_name_list = [],
                msg_if = self.msg_if,
                node_if = self.node_if
    )

    self.track_process_if = ProcessIF(process_name = self.track_process_name,
                process_group = self.node_name,
                process_description = 'Pan tilt track mode',
                process_data_dict = self.track_process_data,
                process_controls_dict = self.track_process_controls,
                process_status_msg = None,
                show_controls = True,
                show_data = True,
                has_enable = True,
                enabled = self.tracking_enabled,
                enableCb = self.trackEnableCb,
                log_name = None,
                log_name_list = [],
                msg_if = self.msg_if,
                node_if = self.node_if
    )

    self.stab_process_if = ProcessIF(process_name = self.stab_process_name,
                process_group = self.node_name,
                process_description = 'Pan tilt stabilize mode',
                process_data_dict = self.stab_process_data,
                process_controls_dict = self.stab_process_controls,
                process_status_msg = None,
                show_controls = True,
                show_data = True,
                has_enable = True,
                enabled = self.stabilize_enabled,
                enableCb = self.stabEnableCb,
                log_name = None,
                log_name_list = [],
                msg_if = self.msg_if,
                node_if = self.node_if
    )


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
    nepi_sdk.start_timer_process(1, self.processCb, oneshot = True)
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
    # self.pushSpeedRatios()

    # Readiness follows the source connections, which move on their own. A
    # process that was enabled while ready and is no longer ready is dropped
    # here rather than left reporting an enable nothing can act on.
    self.auditProcessEnables()

    cur_time = nepi_utils.get_time()
    elapsed = cur_time - self.last_targets_time
    if elapsed > self.WATCHDOG_TARGETS_TIMEOUT:
        self.targets_lock.acquire()
        self.targets_dict_list = []
        self.targets_lock.release()

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

  # def pushSpeedRatios(self):
  #   # A newly connected device knows nothing of the ratios this node restored
  #   # from config, so push them once per connection rather than every cycle.
  #   if self.pantilt_connected == False:
  #     self.last_speed_ratios_pushed = None
  #     return
  #   ratios = [self.speed_ratio, self.pan_speed_ratio, self.tilt_speed_ratio]
  #   if ratios == self.last_speed_ratios_pushed:
  #     return
  #   self.pantilt_connect_if.set_speed_ratio(self.speed_ratio)
  #   self.pantilt_connect_if.set_pan_speed_ratio(self.pan_speed_ratio)
  #   self.pantilt_connect_if.set_tilt_speed_ratio(self.tilt_speed_ratio)
  #   self.last_speed_ratios_pushed = ratios

  def panTiltCb(self, pan_deg, tilt_deg):
    self.pt_position = [pan_deg, tilt_deg]

  def stopPanCb(self):
    self.pan_goto = UNSET_VALUE

  def stopTiltCb(self):
    self.tilt_goto = UNSET_VALUE

  def targetsCb(self, targets_dict):
    #self.msg_if.pub_info("Targets callback got new targets mgs: " + str(targets_dict), throttle_s = 5)
    self.last_targets_time = nepi_utils.get_time()
    targets = targets_dict['data']['targets']
    # Normalized to a list here so the targets watchdog in updaterCb and the
    # consumers in processCb can treat targets_dict_list as a list without
    # re-checking. The earlier version of this callback appended to the same
    # list it was iterating, which never terminated.
    if targets is None:
      targets = []
    self.targets_lock.acquire()
    self.targets_dict_list = [t for t in targets if t is not None]
    self.targets_lock.release()



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

  def setPanPosRatioCb(self, msg):
    self.setPanPosRatio(msg.data)

  def setPanPosRatio(self, pos_ratio):
    ratio = self.clampRatio(pos_ratio)
    if ratio is None:
      return
    if self.pantilt_connected == False:
      self.msg_if.pub_warn("No pan tilt device connected; ignoring pan ratio command")
      return
    if self.getPanControlDisabled() == True:
      self.msg_if.pub_warn("An auto mode owns the pan axis; ignoring pan ratio command")
      return
    self.msg_if.pub_info("Sending pan ratio command: " + str(ratio))
    # pan_goto stays in degrees and is left alone here. Converting the ratio
    # would mean duplicating the device's soft stop mapping in this node, and
    # the acted-on goal comes back in pantilt_status_msg.pan_goal_deg anyway.
    self.pantilt_connect_if.goto_pan_ratio(ratio)
    self.publish_status()

  def setTiltPosRatioCb(self, msg):
    self.setTiltPosRatio(msg.data)

  def setTiltPosRatio(self, pos_ratio):
    ratio = self.clampRatio(pos_ratio)
    if ratio is None:
      return
    if self.pantilt_connected == False:
      self.msg_if.pub_warn("No pan tilt device connected; ignoring tilt ratio command")
      return
    if self.getTiltControlDisabled() == True:
      self.msg_if.pub_warn("An auto mode owns the tilt axis; ignoring tilt ratio command")
      return
    self.msg_if.pub_info("Sending tilt ratio command: " + str(ratio))
    self.pantilt_connect_if.goto_tilt_ratio(ratio)
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

  # Two entry points reach the same state. This node's own set_<mode>_enable
  # topics are what the app panel toggles publish to; each ProcessIF also
  # advertises <process namespace>/set_enable, which is what the generic process
  # panel's Enable toggle publishes to. Both funnel through setProcessEnable so
  # the app status and the four ProcessStatus messages cannot disagree.
  #
  # What an enable actually does today: it sets the mode flags that
  # nepi_auto_pt.pt_auto_2 reads, and it hands the axes to auto control. The
  # controller loop itself is not wired yet -- nothing in this node builds the
  # auto_data_dict it needs (navpose feed, mount transforms, per-axis servo
  # state) -- so an enabled process reports enabled and NOT running, with
  # PROCESS_NOT_RUNNING_MSG saying why. Wiring the loop is what flips running.

  def setAutoEnableCb(self, msg):
    self.setProcessEnable(AUTO_PROCESS, msg.data)

  def setScanningEnableCb(self, msg):
    self.setProcessEnable(SCAN_PROCESS, msg.data)

  def setTrackingEnableCb(self, msg):
    self.setProcessEnable(TRACK_PROCESS, msg.data)

  def setStabilizeEnableCb(self, msg):
    self.setProcessEnable(STAB_PROCESS, msg.data)

  # The four ProcessIF enable callbacks. Each returns the state actually
  # adopted, which is what the process panel then reports -- an enable that was
  # refused falls the toggle back instead of leaving it green.
  def autoEnableCb(self, enabled):
    return self.setProcessEnable(AUTO_PROCESS, enabled)

  def scanEnableCb(self, enabled):
    return self.setProcessEnable(SCAN_PROCESS, enabled)

  def trackEnableCb(self, enabled):
    return self.setProcessEnable(TRACK_PROCESS, enabled)

  def stabEnableCb(self, enabled):
    return self.setProcessEnable(STAB_PROCESS, enabled)

  def getProcessIf(self, process_key):
    return {
        AUTO_PROCESS: self.auto_process_if,
        SCAN_PROCESS: self.scan_process_if,
        TRACK_PROCESS: self.track_process_if,
        STAB_PROCESS: self.stab_process_if
    }.get(process_key, None)

  def getProcessParamName(self, process_key):
    return {
        AUTO_PROCESS: 'auto_enabled',
        SCAN_PROCESS: 'scanning_enabled',
        TRACK_PROCESS: 'tracking_enabled',
        STAB_PROCESS: 'stabilize_enabled'
    }.get(process_key, None)

  def getProcessEnabled(self, process_key):
    return {
        AUTO_PROCESS: self.auto_enabled,
        SCAN_PROCESS: self.scanning_enabled,
        TRACK_PROCESS: self.tracking_enabled,
        STAB_PROCESS: self.stabilize_enabled
    }.get(process_key, False)

  def getProcessReady(self, process_key):
    # Every mode needs the supervisor: a mode flag means nothing to a controller
    # that is not enabled, so a mode is not ready unless auto is ready too.
    if process_key == AUTO_PROCESS:
      return self.getAutoReady()
    if self.getAutoReady() == False:
      return False
    return {
        SCAN_PROCESS: self.getScanningReady,
        TRACK_PROCESS: self.getTrackingReady,
        STAB_PROCESS: self.getStabilizeReady
    }.get(process_key, lambda: False)()

  def storeProcessEnabled(self, process_key, enabled):
    if process_key == AUTO_PROCESS:
      self.auto_enabled = enabled
    elif process_key == SCAN_PROCESS:
      self.scanning_enabled = enabled
    elif process_key == TRACK_PROCESS:
      self.tracking_enabled = enabled
    elif process_key == STAB_PROCESS:
      self.stabilize_enabled = enabled
    else:
      return
    param_name = self.getProcessParamName(process_key)
    if param_name is not None:
      self.setParam(param_name, enabled)
    process_if = self.getProcessIf(process_key)
    if process_if is not None:
      # set_process_enable_state, not set_process_enable: the decision is
      # already made here, and going back through the IF's own setter would
      # re-enter this method through the enable callback.
      process_if.set_process_enable_state(enabled)

  def setProcessEnable(self, process_key, enabled):
    enabled = (enabled == True)
    if process_key not in PROCESS_KEYS:
      self.msg_if.pub_warn("Unknown process: " + str(process_key))
      return False

    if enabled == True and self.getProcessReady(process_key) == False:
      self.msg_if.pub_warn("Process not ready; ignoring enable: " + str(process_key))
      # Re-report so a toggle that was clicked optimistically falls back.
      self.publishProcessRunStates()
      self.publish_status()
      return False

    self.msg_if.pub_info("Setting process enable: " + str(process_key) + " to: " + str(enabled))
    self.storeProcessEnabled(process_key, enabled)

    if enabled == True:
      # pt_auto_2 forces the lower-priority modes off anyway and writes that
      # back; clearing them here keeps the reported state from disagreeing.
      for cleared_key in PROCESS_CLEARS.get(process_key, []):
        if self.getProcessEnabled(cleared_key) == True:
          self.msg_if.pub_info("Clearing process, superseded by " + str(process_key) + ": " + str(cleared_key))
          self.storeProcessEnabled(cleared_key, False)
    elif process_key == AUTO_PROCESS:
      # Dropping the supervisor drops every mode with it. Leaving a mode
      # enabled under a disabled supervisor reports an armed state that
      # nothing can act on.
      for mode_key in MODE_PROCESS_KEYS:
        if self.getProcessEnabled(mode_key) == True:
          self.msg_if.pub_info("Clearing process, auto supervisor disabled: " + str(mode_key))
          self.storeProcessEnabled(mode_key, False)

    self.applyAxisOwnership()
    self.publishProcessRunStates()
    self.publish_status()
    return self.getProcessEnabled(process_key)

  def getAutoModeActive(self):
    if self.auto_enabled == False:
      return False
    for mode_key in MODE_PROCESS_KEYS:
      if self.getProcessEnabled(mode_key) == True:
        return True
    return False

  def applyAxisOwnership(self):
    # An axis is auto-owned only while the supervisor is enabled AND some mode
    # is selected; otherwise it goes back to manual. These four flags are what
    # gate every manual pan/tilt command in this node and what the RUI reads to
    # grey out its sliders, so they are the whole visible effect of an enable
    # until the control loop is wired.
    auto_active = self.getAutoModeActive()
    was_auto = (self.pan_control_auto_enabled or self.tilt_control_auto_enabled)

    self.pan_control_auto_enabled = auto_active
    self.tilt_control_auto_enabled = auto_active
    self.pan_control_manaul_enabled = (auto_active == False)
    self.tilt_control_manaul_enabled = (auto_active == False)

    if auto_active == False and was_auto == True:
      # Releasing the axes without stopping would leave the device running out
      # the last auto command with nothing driving it.
      self.stopPanTilt()

  def stopPanTilt(self):
    if self.pantilt_connected == False or self.pantilt_connect_if is None:
      return
    try:
      self.pantilt_connect_if.stop_moving()
    except Exception as e:
      self.msg_if.pub_warn("Failed to stop pan tilt motion: " + str(e))
    self.pan_goto = UNSET_VALUE
    self.tilt_goto = UNSET_VALUE

  def publishProcessRunStates(self):
    # running is what is actually executing, which is not the same as enabled.
    # Until the pt_auto_2 loop is wired, nothing runs, and each enabled process
    # says so rather than showing a green Running indicator for a loop that
    # does not exist. Wiring the loop is what makes getProcessRunning return
    # something other than False.
    for process_key in PROCESS_KEYS:
      process_if = self.getProcessIf(process_key)
      if process_if is None:
        continue
      enabled = self.getProcessEnabled(process_key)
      running = self.getProcessRunning(process_key)
      if running == True:
        msg_str = ''
      elif enabled == True:
        msg_str = PROCESS_NOT_RUNNING_MSG
      elif self.getProcessReady(process_key) == False:
        msg_str = 'Not ready. Required source not connected.'
      else:
        msg_str = ''
      process_if.set_process_running(running, msg_str = msg_str)

  def getProcessRunning(self, process_key):
    # No control loop is wired yet, so nothing runs. This is the single place
    # to change when nepi_auto_pt.pt_auto_2 is driven from processCb.
    return False

  def auditProcessEnables(self):
    # Auto first: dropping the supervisor clears the modes in the same pass, so
    # auditing it first avoids reporting a mode as independently lost.
    dropped = False
    for process_key in PROCESS_KEYS:
      if self.getProcessEnabled(process_key) == False:
        continue
      if self.getProcessReady(process_key) == True:
        continue
      self.msg_if.pub_warn("Process no longer ready; disabling: " + str(process_key))
      self.setProcessEnable(process_key, False)
      dropped = True
    if dropped == False:
      # setProcessEnable already did both on the drop path.
      self.applyAxisOwnership()
      self.publishProcessRunStates()

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

  def getAutoReady(self):
    # The supervisor needs the device it controls and nothing else. The modes
    # layer their own extra requirements on top of this.
    return self.pantilt_connected == True and self.pantilt_connect_if is not None

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
    return (self.pan_control_manaul_enabled == False and self.pan_control_auto_enabled == False)

  def getTiltControlDisabled(self):
    return (self.tilt_control_manaul_enabled == False and self.tilt_control_auto_enabled == False)

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
      self.auto_enabled = self.node_if.get_param('auto_enabled')
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

      # The ProcessIFs come up before the params are restored, so each one is
      # told the restored enable state here. This node owns that state; the
      # interfaces only report it.
      for process_key in PROCESS_KEYS:
        process_if = self.getProcessIf(process_key)
        if process_if is not None:
          process_if.init()
          process_if.set_process_enable_state(self.getProcessEnabled(process_key))

      if self.auto_process_if is not None:
        self.applyAxisOwnership()
        self.publishProcessRunStates()

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
  ## Auto Process Udpater

  def processCb(self, timer):
    
    start_time = nepi_utils.get_time()
    #####################
    self.targets_lock.acquire()
    targets_dict_list = copy.deepcopy(self.targets_dict_list)
    self.targets_lock.release()
    track_dict = None 




    #####################
    if targets_dict_list is not None and len(targets_dict_list) > 0:
      track_dict = targets_dict_list[0]
    # if len(targets_dict_list) == 0 or self.track_process_if is None:
    #   return
    # [best_target, tracking_dict] = nepi_targets_track.get_best_from_targets(
    #                                     targets_dict_list,
    #                                     tracking_dict = self.tracking_dict)
    # self.tracking_dict = tracking_dict
    # if best_target is None:
    #   return
    # try:
    #   target_msg = nepi_sdk.convert_dict2msg(TARGET_MSG_TYPE, best_target)
    # except Exception as e:
    #   self.msg_if.pub_warn("Failed to rebuild target message: " + str(e))
    #   return
    # if target_msg is None:
    #   return


    #####################
    if track_dict is not None and self.image_connect_if is not None:
      track_msg = Track()
      track_msg.timestamp = nepi_utils.get_time()
      track_msg.process_name = self.node_name
      track_msg.process_namespace = self.node_namespace
      track_msg.source_topic = self.image_connect_if.get_namespace()
      track_msg.source_timestamp = nepi_utils.get_time()
      target_msg = nepi_sdk.convert_dict2msg(TARGET_MSG_TYPE, track_dict)
      track_msg.target = target_msg
      #self.msg_if.pub_info("Publishing track mgs: " + str(track_msg), throttle_s = 5)
      self.node_if.publish_pub('track_pub', track_msg)
    
    max_hz = self.max_process_rate_hz
    if max_hz < 1:
      max_hz = 1
    process_delay = (float(1) / max_hz) - (nepi_utils.get_time() - start_time)
    next_process_delay = max(0.01, process_delay)
    # Re-arm this loop, not updaterCb. Re-arming the updater here ran the
    # process body exactly once and then drove the 1 Hz updater at the process
    # rate instead.
    nepi_sdk.start_timer_process(next_process_delay, self.processCb, oneshot = True)


  ###################
  ## Status Publishers

  def statusPublishCb(self, timer):
    self.publish_status(check = False)


  def publish_status(self, check = True):
    """Publish the current AutoTurretStatus on the node's status topic.

    Called on the status timer and after every accepted command so the RUI and
    the child image publisher node both see a change immediately.
    """
    if self.node_if is None:
      return

    last_status_msg = copy.deepcopy(self.status_msg)

    self.status_msg.max_process_rate_hz = self.max_process_rate_hz
    self.status_msg.max_image_pub_rate_hz = self.max_image_pub_rate_hz

    self.status_msg.auto_select_enabled = self.auto_select_enabled


    # Each connect IF is None until its source is selected, so the local has to
    # be seeded before the guard and the empty fallback has to be the field's
    # own type -- a DevicePTXStatus in the image/targets/navpose slots fails
    # serialization on publish. The connected flags come from the cached
    # self.*_connected sampled in updaterCb, not from a check_connection() call
    # on the status path.
    pantilt_status_msg = None
    if self.pantilt_connect_if is not None:
      self.pantilt_connect_if.set_auto_connect_enable(self.auto_select_enabled)
      self.status_msg.selected_pantilt_topic = self.pantilt_connect_if.get_namespace()
      pantilt_status_msg = self.pantilt_connect_if.get_status_msg()
    if pantilt_status_msg is None:
      self.status_msg.selected_pantilt_topic = "None"
      pantilt_status_msg = DevicePTXStatus()
    self.status_msg.pantilt_connected = self.pantilt_connected
    self.status_msg.pantilt_status_msg = pantilt_status_msg

    image_status_msg = None
    if self.image_connect_if is not None:
      self.image_connect_if.set_auto_connect_enable(self.auto_select_enabled)
      self.status_msg.selected_image_topic = self.image_connect_if.get_namespace()
      image_status_msg = self.image_connect_if.get_status_msg()
    if image_status_msg is None:
      self.status_msg.selected_image_topic = "None"
      image_status_msg = ImageStatus()
    self.status_msg.image_connected = self.image_connected
    self.status_msg.image_status_msg = image_status_msg

    targets_status_msg = None
    if self.targets_connect_if is not None:
      self.targets_connect_if.set_auto_connect_enable(self.auto_select_enabled)
      self.status_msg.selected_targets_topic = self.targets_connect_if.get_namespace()
      targets_status_msg = self.targets_connect_if.get_status_msg()
    if targets_status_msg is None:
      self.status_msg.selected_targets_topic = "None"
      targets_status_msg = TargetingStatus()
    self.status_msg.targets_connected = self.targets_connected
    self.status_msg.targets_status_msg = targets_status_msg

    navpose_status_msg = None
    if self.navpose_connect_if is not None:
      self.navpose_connect_if.set_auto_connect_enable(self.auto_select_enabled)
      self.status_msg.selected_navpose_topic = self.navpose_connect_if.get_namespace()
      navpose_status_msg = self.navpose_connect_if.get_status_msg()
    if navpose_status_msg is None:
      self.status_msg.selected_navpose_topic = "None"
      navpose_status_msg = NavPoseStatus()
    self.status_msg.navpose_connected = self.navpose_connected
    self.status_msg.navpose_status_msg = navpose_status_msg

    self.status_msg.auto_ready = self.getAutoReady()
    self.status_msg.auto_enabled = self.auto_enabled
    self.status_msg.auto_process_namespace = self.auto_process_namespace

    self.status_msg.scanning_ready = self.getProcessReady(SCAN_PROCESS)
    self.status_msg.scanning_enabled = self.scanning_enabled
    self.status_msg.scan_process_namespace = self.scan_process_namespace


    self.status_msg.tracking_ready = self.getProcessReady(TRACK_PROCESS)
    self.status_msg.tracking_enabled = self.tracking_enabled
    self.status_msg.track_process_namespace = self.track_process_namespace


    self.status_msg.stabilize_ready = self.getProcessReady(STAB_PROCESS)
    self.status_msg.stabilize_enabled = self.stabilize_enabled
    self.status_msg.stab_process_namespace = self.stab_process_namespace


    self.status_msg.pan_control_manaul_enabled = self.pan_control_manaul_enabled
    self.status_msg.tilt_control_manaul_enabled = self.tilt_control_manaul_enabled

    self.status_msg.pan_control_auto_enabled = self.pan_control_auto_enabled
    self.status_msg.tilt_control_auto_enabled = self.tilt_control_auto_enabled

    self.status_msg.pan_control_disabled = self.getPanControlDisabled()
    self.status_msg.tilt_control_disabled = self.getTiltControlDisabled()


    self.status_msg.image_pub_topic = self.img_pub_topic

    self.status_msg.show_full_screen = self.show_full_screen
    self.status_msg.show_targets_enabled = self.show_targets_enabled
    self.status_msg.show_track_enabled = self.show_track_enabled
    self.status_msg.show_crosshair_enabled = self.show_crosshair_enabled
    self.status_msg.crosshair_offset_degs = self.crosshair_offset_degs

    if last_status_msg != self.status_msg and check == True:
      self.node_if.publish_pub('status_pub', self.status_msg)


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
