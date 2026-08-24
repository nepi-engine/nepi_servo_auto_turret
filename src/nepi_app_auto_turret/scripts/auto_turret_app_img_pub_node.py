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


import os
import copy
import numpy as np
import math
import cv2
import threading


from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_img

from sensor_msgs.msg import Image

from nepi_interfaces.msg import ImageStatus
from nepi_interfaces.msg import Targets,Track

from nepi_app_auto_turret.msg import AutoTurretStatus

from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF
from nepi_api.system_if import SaveDataIF
from nepi_api.data_if import ColorImageIF


WATCHDOG_DELAY = 60
WATCHDOG_TIMEOUT = 3
WATCHDOG_IMAGE_TIMEOUT = 1
WATCHDOG_TARGETS_TIMEOUT = 1
WATCHDOG_TRACK_TIMEOUT = 1

OVERLAY_CROSSHAIR_COLOR = (0,255, 0)
OVERLAY_TARGETS_COLOR = (0, 0, 0)
OVERLAY_TRACK_COLOR = (0, 0, 0)

# Label font, hoisted out of the draw loop. Nothing about it varies per frame.
OVERLAY_FONT = cv2.FONT_HERSHEY_DUPLEX
OVERLAY_FONT_COLOR = (255, 255, 255)
OVERLAY_LINE_TYPE = cv2.LINE_AA

class AutoTurretImgPub:

    AUTO_TURRET_IMG_DATA_PRODUCT = 'color_image'

    DATA_PRODUCTS = [AUTO_TURRET_IMG_DATA_PRODUCT]

    # Never subscribe to our own overlay outputs as input image sources; skip
    # these product basenames even if the parent process's selected_sources
    # lists them (they resolve as real topics under the image namespace).
    OUTPUT_IMG_PRODUCTS = [AUTO_TURRET_IMG_DATA_PRODUCT]

    node_if = None
    save_data_if = None

    selected_image_topic = 'None'
    last_image_time = 0

    img_if = None
    img_node_dict = dict()
    img_node_lock = threading.Lock()

    img_info_dict = dict()
    img_info_lock = threading.Lock()

    targets_dict_list = dict()
    targets_lock = threading.Lock()
    last_targets_time = 0
    show_targets_enabled = False

    track_dict = dict()
    track_lock = threading.Lock()
    last_track_time = 0
    show_track_enabled = False

    crosshair_offset_degs = [0,0]
    show_crosshair_enabled = False

    # Per-frame-size render constants, built once per size instead of once per
    # frame. Written and read only by the single render thread.
    font_dims_cache = dict()
    flat_color_cache = dict()

    state_str_msg = 'Loading'

    clear_det_time = 1.0

    last_status_time = None

    data_products = DATA_PRODUCTS

    min_range_m = 0.0
    max_range_m = 100.0

    has_color_image = False




    overlay_labels = True
    overlay_range_bearing = True

    DEFAULT_NODE_NAME = "auto_turret_img_pub"  # Can be overwritten by launch command

    connected = False

    watchdog_timeout = None

    def __init__(self):
        ####  NODE INIT SETUP ####
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
        # Init Class Variables

        # This node is launched by the parent auto_turret node as
        # <parent node namespace>_img_pub, so stripping the suffix recovers the
        # parent's namespace.
        self.process_namespace = self.node_namespace.replace("_img_pub", "")

        self.msg_if.pub_info("Starting with Process Namespace: " + str(self.process_namespace))

        self.status_msg = AutoTurretStatus()
        self.enabled = False
        self.state_str_msg = "Unknown"
        self.max_image_pub_rate_hz = 10
        self.use_last_image = True

        self.imaging_enabled = True


        ##############################
        # Create NodeClassIF Class

        # Configs Dict ########################
        # This node holds no configuration of its own -- every setting it
        # honors arrives on the parent's status message.
        self.CONFIGS_DICT = None

        # Params Config Dict ####################
        self.PARAMS_DICT = None

        # Services Config Dict ####################
        self.SRVS_DICT = None

        # Pubs Config Dict ####################
        self.PUBS_DICT = None

        # Subs Config Dict ####################
        self.SUBS_DICT = {
            'auto_turret_status_sub': {
                'msg': AutoTurretStatus,
                'namespace': self.process_namespace,
                'topic': 'status',
                'qsize': 10,
                'callback': self.statusCb,
                'callback_args': ()
            },
            'auto_turret_track_sub': {
                'msg': Track,
                'namespace': self.process_namespace,
                'topic': 'track',
                'qsize': 10,
                'callback': self.trackCb,
                'callback_args': ()
            },

            
        }

        # Create Node Class ####################
        self.node_if = NodeClassIF(
                        configs_dict = self.CONFIGS_DICT,
                        params_dict = self.PARAMS_DICT,
                        services_dict = self.SRVS_DICT,
                        pubs_dict = self.PUBS_DICT,
                        subs_dict = self.SUBS_DICT,
                        msg_if = self.msg_if
                        )
        self.node_if.wait_for_ready()

        ###############################
        # Create System IFs

        # Setup Save Data IF. pub_status is False because the parent node owns
        # the save_data status topic on this same namespace; this instance is
        # only here so the image data product honors the same save commands.
        factory_data_rates = {}
        for d in self.data_products:
            factory_data_rates[d] = [1.0, 0.0, 100]

        self.save_data_if = SaveDataIF(namespace = self.process_namespace,
                        data_products = self.data_products,
                        pub_status = False,
                        factory_rate_dict = factory_data_rates,
                        msg_if = self.msg_if,
                        node_if = self.node_if
                        )
        nepi_sdk.sleep(1)



        # Create auto_turret image publisher
        img_pub_topic = os.path.join(self.node_namespace, self.AUTO_TURRET_IMG_DATA_PRODUCT)
        self.img_if = ColorImageIF(namespace = img_pub_topic,
                        data_product = self.AUTO_TURRET_IMG_DATA_PRODUCT,
                        data_source_description = 'image',
                        data_ref_description = 'image',
                        perspective = 'pov',
                        save_data_if = self.save_data_if,
                        init_overlay_text_list = [],
                        live_adjustments_disabled = True,
                        aspect_adjustment_disabled = True,
                        log_name = self.AUTO_TURRET_IMG_DATA_PRODUCT,
                        log_name_list = [],
                        msg_if = self.msg_if)

        nepi_sdk.sleep(1)


        ##########################
        # Complete Initialization

        # Start Timer Processes
        nepi_sdk.start_timer_process((1.0), self.updaterCb, oneshot = True)
        self.last_status_time = nepi_utils.get_time()
        nepi_sdk.start_timer_process(1, self.watchdogCb, oneshot = True)
        nepi_sdk.on_shutdown(self.shutdownCb)

        #########################################################
        ## Initiation Complete
        self.msg_if.pub_info("Initialization Complete")
        # Spin forever
        nepi_sdk.spin()
        #########################################################


    ###############################
    # Class Private Methods
    ###############################

    def getImgInfoDict(self):
        self.img_info_lock.acquire()
        img_info_dict = copy.deepcopy(self.img_info_dict)
        self.img_info_lock.release()
        return img_info_dict


    def createImgInfoDict(self, source_topic):
        img_info_dict = dict()
        img_info_dict['source_topic'] = source_topic
        img_info_dict['active'] = True
        img_info_dict['img_connected'] = False
        img_info_dict['img_published'] = False
        img_info_dict['status_dict'] = None
        

        img_info_dict['connected'] = False
        img_info_dict['publishing'] = False
        img_info_dict['get_latency_time'] = 0
        img_info_dict['pub_latency_time'] = 0
        img_info_dict['process_time'] = 0
        img_info_dict['last_img_time'] = 0


        return img_info_dict


    ###############.########################
    # Per-cycle result snapshots


    def getSourceResult(self, source_topic):
        # One reference, one read, no copy. Entries are immutable once stored, so
        # the maps do not have to be copied out of the way of the next cycle's
        # writer the way they did when the render read them field by field.
        result_dict = self.targets_dict_list.get(source_topic, None)
        if result_dict is None:
            return self.createResultDict()
        return result_dict

    def setSourceResult(self, source_topic, result_dict):
        self.targets_lock.acquire()
        self.targets_dict_list[source_topic] = result_dict
        self.targets_lock.release()

    def clearSourceResult(self, source_topic):
        self.targets_lock.acquire()
        if source_topic in self.targets_dict_list.keys():
            self.targets_dict_list[source_topic] = self.createResultDict()
        self.targets_lock.release()

    ###############.########################
    # Render handoff

    def setRenderSlot(self, source_topic, timestamp, img_msg):
        self.render_slot_lock.acquire()
        self.render_slot_dict[source_topic] = (timestamp, img_msg)
        self.render_slot_lock.release()

    def getRenderSlotTopics(self):
        self.render_slot_lock.acquire()
        source_topics = list(self.render_slot_dict.keys())
        self.render_slot_lock.release()
        return source_topics

    def popRenderSlot(self, source_topic):
        # Taking the frame out is what makes this a slot and not a queue: if the
        # render is slower than the source, the frames that arrive in between
        # overwrite each other and only the newest is ever drawn.
        self.render_slot_lock.acquire()
        slot = self.render_slot_dict.pop(source_topic, None)
        self.render_slot_lock.release()
        return slot

    def clearRenderSlot(self, source_topic):
        self.render_slot_lock.acquire()
        if source_topic in self.render_slot_dict.keys():
            del self.render_slot_dict[source_topic]
        self.render_slot_lock.release()

    def updaterCb(self, timer):
        selected_image_topic = copy.deepcopy(self.selected_image_topic)

        # Update Image subscribers
        source_topic = nepi_sdk.find_topic(selected_image_topic, exact = True)
        if source_topic != '':
            subscribe = False
            if self.img_info_dict is None:
                subscribe = True
            elif self.img_info_dict['source_topic'] != source_topic:
                self.msg_if.pub_info('Will unsubscribe from image topic: ' + self.img_info_dict['source_topic'])
                self.unsubscribeImgTopic()
                subscribe = True
            if subscribe == True:
                self.msg_if.pub_info('Will subscribe to image topic: ' + source_topic)
                self.subscribeImgTopic(source_topic)

        # Update Image Subs purge list
        purge_source = False
        if self.img_info_dict is not None and source_topic == '':
            purge_source = True
        if purge_source == True:
            self.msg_if.pub_info('Will unsubscribe from image topic: ' + selected_image_topic)
            self.unsubscribeImgTopic()

        nepi_sdk.start_timer_process((1), self.updaterCb, oneshot = True)

    def watchdogCb(self, timer):
        cur_time = nepi_utils.get_time()
        elapsed = cur_time - self.last_status_time
        if self.watchdog_timeout is None:
            self.watchdog_timeout = WATCHDOG_TIMEOUT
            nepi_sdk.sleep(WATCHDOG_DELAY)
        else:
            if elapsed > WATCHDOG_TIMEOUT:
                msg = "Lost connection to parent node status msg.  Shutting down"
                self.msg_if.pub_warn(msg)
                nepi_sdk.signal_shutdown(msg)
                return

        cur_time = nepi_utils.get_time()
        elapsed = cur_time - self.last_targets_time
        if elapsed > WATCHDOG_TARGETS_TIMEOUT:
            self.targets_dict_list = None

        cur_time = nepi_utils.get_time()
        elapsed = cur_time - self.last_track_time
        if elapsed > WATCHDOG_TRACK_TIMEOUT:
            self.targets_dict_list = None

        nepi_sdk.start_timer_process(1, self.watchdogCb, oneshot = True)

    def subscribeImgTopic(self, source_topic):
        if source_topic == "None" or source_topic == "":
            return False



        img_info_dict = self.getImgInfoDict()
        if img_info_dict is not None:
            if img_info_dict['active'] == True:
                return False
            self.img_node_lock.acquire()
            if self.img_node_dict is None:
                self.img_node_dict = dict()
                self.img_node_dict['img_pub'] = nepi_sdk.create_publisher(img_pub_topic, Image, queue_size = 1, log_name_list = [])
                nepi_sdk.sleep(1)
                self.img_node_dict['img_sub'] = nepi_sdk.create_subscriber(source_topic, Image, self.imageCb, queue_size = 1, callback_args = (source_topic), log_name_list = [])
                self.img_node_dict['targets_sub'] = nepi_sdk.create_subscriber(source_topic + '/targets', Targets, self.targetsCb, queue_size = 1, callback_args = (source_topic), log_name_list = [])
                img_status_topic = nepi_sdk.create_namespace(source_topic, 'status')
                self.img_node_dict['img_status_sub'] = nepi_sdk.create_subscriber(img_status_topic, ImageStatus, self.imageStatusCb, queue_size = 1, callback_args = (source_topic), log_name_list = [])
            self.img_node_lock.release()
            self.img_info_lock.acquire()
            self.img_info_dict['active'] = True
            self.img_info_lock.release()
        else:
            self.img_info_lock.acquire()
            self.img_info_dict = self.createImgInfoDict(source_topic)
            self.img_info_lock.release()

        return True

    def unsubscribeImgTopic(self):

        if self.img_info_dict is None:
            return False

        self.msg_if.pub_info('Unsubscribing from image topic: ' + self.img_info_dict['source_topic'])

        self.img_node_lock.acquire()
        if self.img_node_dict is not None:
            if self.img_node_dict['img_sub'] is not None:
                self.img_node_dict['img_sub'].unregister()
            if self.img_node_dict['img_status_sub'] is not None:
                self.img_node_dict['img_status_sub'].unregister()
            if self.img_node_dict['img_pub'] is not None:
                self.img_node_dict['img_pub'].unregister()
            for product in self.SEGMENT_IMG_PRODUCTS:
                segment_pub = self.img_node_dict.get(product['pub_key'], None)
                if segment_pub is not None:
                    segment_pub.unregister()
                segment_if = self.img_node_dict.get(product['if_key'], None)
                if segment_if is not None:
                    segment_if.unregister_pubs()
            nepi_sdk.sleep(1)
            self.img_node_dict = None
        
        self.img_node_lock.release()

        self.img_info_lock.acquire()
        self.img_info_dict['active'] = False
        self.img_info_dict['status_dict'] = None
        self.img_info_dict['connected'] = False
        self.img_info_dict['publishing'] = False
        self.img_info_dict['img_connected'] = False
        self.img_info_dict['img_published'] = False
        self.img_info_lock.release()

        return True

    def needsImgCheck(self):
        # if_key None asks about every published product for this source; a key
        # asks about that one. The IF's needs_data is a level flag its own timer
        # refreshes from subscriber and save state, so polling it is free.
        needs_img = False
        if self.img_if is not None:
            if self.img_if.needs_data_check() == True:
                needs_img = True
        return needs_img

    def imageStatusCb(self, status_msg, args):
        source_topic = args
        if source_topic not in self.img_info_dict.keys():
            return
        self.img_info_lock.acquire()
        if source_topic in self.img_info_dict.keys():
            status_dict = nepi_sdk.convert_msg2dict(status_msg)
            if self.img_info_dict['status_dict'] is None:
                self.msg_if.pub_info('Connected to image status topic: ' + source_topic + '/status')
            self.img_info_dict['status_dict'] = status_dict
        self.img_info_lock.release()

    def imageCb(self, image_msg, args):
        source_topic = args
        if self.imaging_enabled == False or source_topic != self.selected_image_topic:
            return
    
        if self.img_info_dict['img_connected'] == False:
            self.msg_if.pub_info('Connected to image topic: ' + source_topic)
        self.img_info_dict['img_connected'] = True


        # Both are replaced whole by statusCb and never mutated in place, so a
        # plain read is a consistent read.
        sel_imgs = self.selected_image_topic
        max_image_pub_rate_hz = self.max_image_pub_rate_hz
        if source_topic != self.selected_image_topic or max_image_pub_rate_hz <= 0.01:
            return

        if self.img_info_dict['connected'] == False:
            self.msg_if.pub_info("Got image topic: " + str(source_topic))
        self.img_info_dict['connected'] = True

        # if self.enabled == False or self.state_str_msg != AutoTurretStatus.STATE_PROCESSING:
        #     return

        timestamp = float(image_msg.header.stamp.to_sec())
        self.img_info_dict['get_latency_time'] = (nepi_utils.get_time() - timestamp)

        start_time = nepi_utils.get_time()
        max_image_pub_rate_hz = self.max_image_pub_rate_hz
        if max_image_pub_rate_hz <= 0.01:
            max_image_pub_rate_hz = 0.01
        delay_time = float(1) / max_image_pub_rate_hz


        last_img_time = self.img_info_dict['last_img_time']
        current_time = nepi_utils.get_time()
        if round((current_time - last_img_time), 3) <= delay_time:
            return

        self.img_info_dict['publishing'] = True

        cv2_img = nepi_img.rosimg_to_cv2img(image_msg)

        # status_dict is replaced whole by imageStatusCb and never mutated, so
        # the reference is safe to read without a copy.
        status_dict = self.img_info_dict['status_dict']
        if status_dict is not None:
            width_pixel = status_dict['width_px']
            height_pixel = status_dict['height_px']
            width_deg = status_dict['width_deg']
            height_deg = status_dict['height_deg']
        else:
            width_pixel = 0
            height_pixel = 0
            width_deg = 100
            height_deg = 70

        targets_dict_list = copy.deepcopy(self.targets_dict_list)
        draw_targets = (len(targets_dict_list) > 0 and self.show_targets_enabled == True)

        track_dict = copy.deepcopy(self.track_dict)
        draw_track = (track_dict is not None and self.show_track_enabled == True)
        

        if draw_targets == True:
            if track_dict is not None and draw_track == True:
                    for i, target_dict in enumerate(targets_dict_list):
                        if track_dict == target_dict:
                            del targets_dict_list[i]
            cv2_img = self.applyBoxesOverlay(targets_dict_list, cv2_img, OVERLAY_TARGETS_COLOR)

        if draw_track == True:
            cv2_img = self.applyBoxesOverlay(track_dict, cv2_img, OVERLAY_TRACK_COLOR)


        
        self.publishImgData(cv2_img,
                            width_deg = width_deg,
                            height_deg = height_deg,
                            timestamp = timestamp,
                            add_overlay_text_list = []
                            )

        if self.img_info_dict['img_published'] == False:
            namespace = self.img_info_dict['pub_namespace']
            self.msg_if.pub_info('Published image topic: ' + os.path.join(namespace, self.AUTO_TURRET_IMG_DATA_PRODUCT))
        self.img_info_dict['img_published'] = True

    def publishImgData(self, cv2_img, encoding = "bgr8", timestamp = None,
                        width_deg = 100,
                        height_deg = 70,
                        add_overlay_text_list = [],):
        if self.imaging_enabled == False:
            return

        # The lock covers looking the publishers up, not publishing through them.
        # Held across the publish it serialized the whole encode -- three products
        # and every source behind one mutex, and any thread that so much as asked
        # whether a product needed data waited behind that. A publisher torn down
        # by unsubscribeImgTopic between the lookup and the publish raises, which
        # is what the try/except below is for.
          
        if self.img_if.ready == False:
            return
        else:
            if self.show_crosshair_enabled == True:
                [x_deg,y_deg] = copy.deepcopy(self.crosshair_offset_degs)
                self.img_if.add_crosshair_degs(x_deg,y_deg,name = 'pos_goal')
                self.img_if.set_crosshairs_enable(True)
            else:
                self.img_if.remove_crosshair('pos_goal')
                self.img_if.set_crosshairs_enable(False)

            self.img_if.publish_cv2_img(cv2_img,
                                encoding = encoding,
                                timestamp = timestamp,
                                width_deg = width_deg,
                                height_deg = height_deg,
                                add_overlay_text_list = add_overlay_text_list
                                )
    

    def getOverlayFontDims(self, cv2_img):
        # Label metrics depend only on the frame size. Same numbers as before,
        # computed once per size rather than once per frame.
        cv2_shape = cv2_img.shape
        key = (cv2_shape[0], cv2_shape[1])
        font_dims = self.font_dims_cache.get(key, None)
        if font_dims is None:
            img_height = cv2_shape[0]
            img_width = cv2_shape[1]
            scale = 1.5e-3 - 0.1e-3 * math.ceil(max([img_height, img_width]) / 700)
            [font_scale, font_thickness] = nepi_img.optimal_font_dims(cv2_img, font_scale = scale, thickness_scale = scale)
            line_thickness = 1 + math.ceil(max([img_height, img_width]) / 2000)
            font_dims = [font_scale, font_thickness, line_thickness]
        return font_dims

    def applyBoxesOverlay(self, boxes_dict_list, cv2_img, default_color):
        # Draws in place, same as applyDepthMapOverlay and for the same reason.
        cv2_shape = cv2_img.shape
        img_size = cv2_shape[:2]

        font = OVERLAY_FONT
        [fontScale, fontThickness, line_thickness] = self.getOverlayFontDims(cv2_img)
        fontColor = OVERLAY_FONT_COLOR
        lineType = OVERLAY_LINE_TYPE

        for box_dict in boxes_dict_list:
            ###### Apply Image Overlays and Publish Image ROS Message
            class_name = box_dict['name']
            xmin = box_dict['xmin']
            ymin = box_dict['ymin']
            xmax = box_dict['xmax']
            ymax = box_dict['ymax']

            if xmin <= 0:
                xmin = 5
            if ymin <= 0:
                ymin = 5
            if xmax >= img_size[1]:
                xmax = img_size[1] - 5
            if ymax >= img_size[0]:
                ymax = img_size[0] - 5

            bot_left_box = (xmin, ymin)
            top_right_box = (xmax, ymax)

            class_color = default_color

            success = False
            try:
                cv2.rectangle(cv2_img, bot_left_box, top_right_box, class_color, thickness = line_thickness)
                success = True
            except Exception as e:
                self.msg_if.pub_warn("Failed to create bounding box rectangle: " + str(e), throttle_s = 5.0)

            if success == False:
                continue

            ## Overlay Text
            overlay_text = ""
            if self.overlay_labels:
                overlay_text = overlay_text + str(class_name) + " "
            if self.overlay_range_bearing:
                rb_text = ''
                if box_dict['range_m'] != -999 and box_dict['range_m'] != '':
                    rb_text = rb_text + str(round(box_dict['range_m'], 1)) + 'm :'
                if box_dict['azimuth_deg'] != -999 and box_dict['elevation_deg'] != -999:
                    rb_text = rb_text + str(round(box_dict['azimuth_deg'], 1)) + 'deg '
                    rb_text = rb_text + str(round(box_dict['elevation_deg'], 1)) + 'deg '
                if len(rb_text) > 0:
                    overlay_text = overlay_text + rb_text

            if len(overlay_text) == 0:
                continue

            text_size = cv2.getTextSize(overlay_text, font, fontScale, fontThickness)
            line_height = text_size[0][1]
            line_width = text_size[0][0]
            x_padding = int(line_height * 0.4)
            y_padding = int(line_height * 0.4)

            center = bot_left_box[0] + int((top_right_box[0] - bot_left_box[0]) / 2)
            bot_left_text = (center + x_padding, ymin - (line_thickness * 2) - y_padding)
            text_bot_left_box = (center - x_padding, bot_left_text[1] + y_padding)
            text_top_right_box = (center + line_width + x_padding, bot_left_text[1] - line_height - y_padding)
            box_color = OVERLAY_TARGETS_COLOR

            try:
                cv2.rectangle(cv2_img, text_bot_left_box, text_top_right_box, box_color, -1)
                cv2.putText(cv2_img, overlay_text,
                    bot_left_text,
                    font,
                    fontScale,
                    fontColor,
                    fontThickness,
                    lineType)
            except Exception as e:
                self.msg_if.pub_warn("Failed to apply overlay label text: " + str(e), throttle_s = 5.0)

        return cv2_img

    def getBoxDict(self, entry_dict):
        return {
            'name': entry_dict.get('name', ''),
            'xmin': entry_dict.get('xmin_pixel', 0),
            'ymin': entry_dict.get('ymin_pixel', 0),
            'xmax': entry_dict.get('xmax_pixel', 0),
            'ymax': entry_dict.get('ymax_pixel', 0),
            'range_m': entry_dict.get('range_m', -999),
            'azimuth_deg': entry_dict.get('azimuth_deg', -999),
            'elevation_deg': entry_dict.get('elevation_deg', -999),
        }

    def targetsCb(self, msg):
        source_topic = msg.source_topic
        if source_topic != self.selected_image_topic:
            return

        current_time = nepi_utils.get_time()
        # msg.targets is an Targets[] array -- convert_msg2dict takes a single
        # message, so convert per entry.
        targets_list = []
        for target_msg in msg.targets:
            targets_list.append(nepi_sdk.convert_msg2dict(target_msg))

        targets_dict_list = []
        for target in targets_list:
            targets_dict_list.append(self.getBoxDict(target))
        self.targets_dict_list = targets_dict_list
        self.last_targets_time = nepi_utils.get_time()

    def trackCb(self, msg):
        self.track_dict = nepi_sdk.convert_msg2dict(msg)
        self.last_track_time = nepi_utils.get_time()

    def statusCb(self, msg):
        self.last_status_time = nepi_utils.get_time()

        self.status_msg = msg.process_status

        self.enabled = self.status_msg.enabled
        self.state_str_msg = self.status_msg.msg_str
        self.max_image_pub_rate_hz = self.status_msg.max_image_pub_rate_hz
        self.use_last_image = self.status_msg.use_last_image
        self.imaging_enabled = self.status_msg.image_pub_enabled

        last_sel_imgs = copy.deepcopy(self.selected_image_topic)
        self.selected_image_topic = msg.selected_image_topic
        if last_sel_imgs != self.selected_image_topic:
            self.msg_if.pub_info("Updating selected images topics: " + str(self.selected_image_topic))

        self.show_targets_enabled = msg.show_targets_enabled
        self.show_track_enabled = msg.show_track_enabled
        self.show_crosshair_enabled = msg.show_crosshair_enabled
        self.crosshair_offset_degs = msg.crosshair_offset_degs


    def shutdownCb(self):
        for source_topic in list(self.img_info_dict.keys()):
            try:
                self.unsubscribeImgTopic(source_topic)
            except Exception:
                pass


#########################################
# Main
#########################################
if __name__ == '__main__':
    AutoTurretImgPub()
