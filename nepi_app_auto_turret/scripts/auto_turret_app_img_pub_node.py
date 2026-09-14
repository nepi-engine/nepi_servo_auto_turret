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
from nepi_interfaces.msg import Targets
from nepi_interfaces.msg import ProcessResultsTrack

from nepi_app_auto_turret.msg import AutoTurretStatus

from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF
from nepi_api.system_if import SaveDataIF
from nepi_api.data_if import ColorImageIF


WATCHDOG_DELAY = 20
WATCHDOG_TIMEOUT = 2
WATCHDOG_IMAGE_TIMEOUT = 1
WATCHDOG_TARGETS_TIMEOUT = 1
WATCHDOG_TRACK_TIMEOUT = 1


class AutoTurretImgPub:

    AUTO_TURRET_IMG_DATA_PRODUCT = 'process_image'

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
    img_node_dict = None
    img_node_lock = threading.Lock()

    img_info_dict = None
    img_info_lock = threading.Lock()

    targets_results_msg = []
    targets_lock = threading.Lock()
    targets_time = 0
    show_targets_enabled = False

    track_results = None
    track_lock = threading.Lock()
    track_results_time = 0
    show_track_enabled = False

    goal_error_degs = [0,0]
    show_goal_enabled = False

    # Per-frame-size render constants, built once per size instead of once per
    # frame. Written and read only by the single render thread.
    font_dims_cache = dict()
    flat_color_cache = dict()


    clear_det_time = 1.0

    last_status_time = None

    data_products = DATA_PRODUCTS

    min_range_m = 0.0
    max_range_m = 100.0

    has_color_image = False

    draw_targets = False
    draw_track = False
    draw_crosshair = False


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

        self.enabled = False
        self.state_str_msg = "Unknown"
        self.max_image_pub_rate_hz = 10
        self.use_last_image = False

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
                'msg': ProcessResultsTrack,
                'namespace': self.process_namespace + '/process_track',
                'topic': 'track',
                'qsize': 10,
                'callback': self.trackResultsCb,
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
        self.img_if = ColorImageIF(namespace = self.process_namespace,
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
        self.img_if.clear_targets()
        self.img_if.clear_crosshairs()

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


    def createImgInfoDict(self, source_topic):
        img_info_dict = dict()
        img_info_dict['source_topic'] = source_topic
        img_info_dict['pub_namespace'] = self.process_namespace
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



    def updaterCb(self, timer):
        source_topic = copy.deepcopy(self.selected_image_topic)

        # Update Image subscrif 
        if source_topic == 'None':
            source_topic = ''
        success = False
        if source_topic != '':
            subscribe = False
            success = True
            if self.img_info_dict is None:
                subscribe = True
            elif self.img_info_dict['source_topic'] != source_topic:
                self.msg_if.pub_info('Will unsubscribe from image topic: ' + self.img_info_dict['source_topic'])
                success = self.unsubscribeImgTopic()
                subscribe = True
            if subscribe == True and success == True:
                self.msg_if.pub_info('Will subscribe to image topic: ' + source_topic)
                success = self.subscribeImgTopic(source_topic)

        # Update Image Subs purge list
        purge_source = False
        if self.img_info_dict is not None and source_topic == '':
            purge_source = True
        if purge_source == True:
            self.msg_if.pub_info('Will unsubscribe from image topic: ' + source_topic)
            success = self.unsubscribeImgTopic()

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
        elapsed = cur_time - self.targets_time
        if elapsed > WATCHDOG_TARGETS_TIMEOUT:
            self.targets_results_dict = None

        cur_time = nepi_utils.get_time()
        elapsed = cur_time - self.track_results_time
        if elapsed > WATCHDOG_TRACK_TIMEOUT:
            self.track_results = None

        nepi_sdk.start_timer_process(1, self.watchdogCb, oneshot = True)

    def subscribeImgTopic(self, source_topic):
        if source_topic == "None" or source_topic == "":
            return False
            
        if self.img_node_dict is None:
            self.img_node_lock.acquire()
            self.img_node_dict = dict()
            self.img_node_dict['img_sub'] = nepi_sdk.create_subscriber(source_topic, Image, self.imageCb, queue_size = 1, callback_args = (source_topic), log_name_list = [])
            self.img_node_dict['img_status_sub'] = nepi_sdk.create_subscriber(source_topic + '/status', ImageStatus, self.imageStatusCb, queue_size = 1, callback_args = (source_topic), log_name_list = [])
            self.img_node_dict['targets_sub'] = nepi_sdk.create_subscriber(source_topic + '/targets', Targets, self.targetsResultsCb, queue_size = 1, callback_args = (source_topic), log_name_list = [])
            self.img_node_lock.release()

        if self.img_info_dict is None:
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
            for key in self.img_node_dict.keys():
                try:
                    self.img_node_dict[key].unregister()
                except:
                    pass
            nepi_sdk.sleep(1)
            self.img_node_dict = None
        
        self.img_node_lock.release()

        self.img_info_lock.acquire()
        self.img_info_dict = None
        # self.img_info_dict['active'] = False
        # self.img_info_dict['status_dict'] = None
        # self.img_info_dict['connected'] = False
        # self.img_info_dict['publishing'] = False
        # self.img_info_dict['img_connected'] = False
        # self.img_info_dict['img_published'] = False
        self.img_info_lock.release()

        return True

    def needsImgCheck(self, source_topic):
        needs_img = False
        if self.imaging_enabled == False or \
                self.img_if.ready == False or \
                source_topic != self.selected_image_topic or \
                self.img_info_dict is None or \
                nepi_sdk.is_shutdown() == True:
            needs_img = False
        else:
            needs_img = self.img_if.needs_data_check()
        return needs_img

    def imageStatusCb(self, status_msg, args):
        source_topic = args
        if self.img_info_dict is None or nepi_sdk.is_shutdown() == True:
            return
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

        if self.img_info_dict['img_connected'] == False:
            self.msg_if.pub_info('Connected to image topic: ' + source_topic)
        self.img_info_dict['img_connected'] = True

        needs_img = self.needsImgCheck(source_topic)
          
        if needs_img == True:


            # Both are replaced whole by statusCb and never mutated in place, so a
            # plain read is a consistent read.
            sel_imgs = self.selected_image_topic
            max_image_pub_rate_hz = self.max_image_pub_rate_hz
            if source_topic != self.selected_image_topic or max_image_pub_rate_hz <= 0.01:
                return

            if self.img_info_dict['connected'] == False:
                self.msg_if.pub_info("Got image topic: " + str(source_topic))
            self.img_info_dict['connected'] = True

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
            image_dict = copy.deepcopy(self.img_info_dict['status_dict'])

            if draw_targets == True:

                targets_results_dict = copy.deepcopy(self.targets_results_dict)
                draw_targets = self.show_targets_enabled
                controls_dict = dict()
                cv2_img = self.process_results_image(cv2_img, image_dict, targets_results_dict, controls_dict)




            # The lock covers looking the publishers up, not publishing through them.
            # Held across the publish it serialized the whole encode -- three products
            # and every source behind one mutex, and any thread that so much as asked
            # whether a product needed data waited behind that. A publisher torn down
            # by unsubscribeImgTopic between the lookup and the publish raises, which
            # is what the try/except below is for.
            





            draw_track = (self.show_track_enabled == True)
            if draw_track == True:


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

                try:
                        [x_deg,y_deg] = [0,0]
                        track_results = copy.deepcopy(self.track_results)
                        if track_results is not None:
                            try:
                                [x_deg,y_deg] = [track_results['azimuth_deg'],track_results['elevation_deg']]
                            except Exception as e:
                                self.msg_if.pub_info('Draw Target Failed: ' + str(track_results) + " with exception: " + str(e), throttle_s = 5)

                        self.img_if.add_target_degs(x_deg,y_deg, name = 'Track Goal', color_rgb = OVERLAY_TRACK_COLOR)


                        if draw_track == True:
                            self.img_if.set_targets_size_ratio(0.4)
                            self.img_if.set_targets_thickness_ratio(0.4)
                            self.img_if.set_targets_text_ratio(0.3)
                            self.img_if.set_overlay_target_degrees(True)
                        else:
                            self.img_if.remove_target('Track Goal')
                        self.img_if.set_targets_enable(draw_track)
                        self.draw_track = draw_track
                except Exception as e:
                    self.msg_if.pub_info('Draw Target Failed: ' + str(track_results) + " with exception: " + str(e), throttle_s = 5)


            draw_crosshair = (self.show_goal_enabled == True)
            try:
                if draw_crosshair == True:
                    [x_deg,y_deg] = copy.deepcopy(self.goal_error_degs)
                    self.img_if.add_crosshair_degs(x_deg,y_deg,name = 'Move Goal', color_rgb = OVERLAY_CROSSHAIR_COLOR)
                if self.draw_crosshair != draw_crosshair:
                    if draw_crosshair == True:
                        self.img_if.set_crosshairs_size_ratio(0.4)
                        self.img_if.set_crosshairs_thickness_ratio(0.4)
                        self.img_if.set_crosshairs_text_ratio(0.3)
                        self.img_if.set_overlay_crosshair_degrees(True)
                    else:
                        self.img_if.remove_crosshair('Move Goal')
                    self.img_if.set_crosshairs_enable(draw_crosshair)
                    self.draw_crosshair = draw_crosshair
            except Exception as e:
                self.msg_if.pub_info('Draw Crosshair Failed: ' + str([x_deg,y_deg]) + " with exception: " + str(e), throttle_s = 5)



            self.img_if.publish_cv2_img(cv2_img,
                                encoding = "bgr8",
                                timestamp = timestamp,
                                width_deg = width_deg,
                                height_deg = height_deg,
                                add_overlay_text_list = []
                                )         


        if self.img_info_dict['img_published'] == False:
            namespace = self.img_info_dict['pub_namespace']
            self.msg_if.pub_info('Published image topic: ' + os.path.join(self.process_namespace, self.AUTO_TURRET_IMG_DATA_PRODUCT))
        self.img_info_dict['img_published'] = True







    #############################
    # Targets Results
    def convert_results_pub_msg2dict(self, results_msg):
        results_dict = nepi_sdk.convert_msg2dict(results_msg)
        return results_dict


OVERLAY_CROSSHAIR_COLOR = (0,255, 0)
OVERLAY_TARGETS_COLOR = (255, 255, 255)
OVERLAY_TRACK_COLOR = (255, 0, 0)

    def process_results_image(self, cv2_img, status_dict, controls_dict, results_dict):
        ##################
        # Get Image Data
        try:
            cv2_img_results = copy.deepcopy(cv2_img)
            cv2_shape = cv2_img.shape
            img_width = cv2_shape[1] 
            img_height = cv2_shape[0] 
        except:
            return cv2_img

        if status_dict is None:
            status_dict = dict()
        width_deg = status_dict.get('width_deg', 100)
        height_deg = status_dict.get('height_deg', 70)

        ##################
        # Get Controls Data
        if controls_dict is None:
            controls_dict = dict()
        overlay_color = controls_dict.get('overlay_color',(0,0,127))
        overlay_font = controls_dict.get('overlay_color',nepi_img.OVERLAY_FONT)
        overlay_font_color = controls_dict.get('overlay_color',nepi_img.OVERLAY_FONT_COLOR)
        overlay_line_type = controls_dict.get('overlay_color',nepi_img.OVERLAY_LINE_TYPE)





        ##################
        # Get Results Data
        if results_dict is None:
            results_dict = dict()       
        targets_list = results_dict.get('targets', [])

        ##################
        # Process Results Image
        for i, target_dict in enumerate(targets_list):
            try:
                img_size = cv2_img.shape[:2]

                # Overlay text data on OpenCV image
                font = overlay_font
                scale = 1.5e-3 - 0.1e-3 * math.ceil(max([img_height, img_width])/700)
                fontScale, fontThickness  = nepi_img.optimal_font_dims(cv2_img,font_scale = scale, thickness_scale = scale, scale_ratio = 0.5) 
                fontColor = (255, 255, 255)
                fontColorBk = (0,0,0)
                lineType = overlay_line_type


                ###### Apply Image Overlays and Publish Image ROS Message
                # Overlay adjusted detection boxes on image 
                class_name = target_dict['name']
                xmin = target_dict['xmin_pixel']
                ymin = target_dict['ymin_pixel']
                xmax = target_dict['xmax_pixel']
                ymax = target_dict['ymax_pixel']

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


                class_color = overlay_color
            
                #self.msg_if.pub_warn("Got Class Color: " + str(class_color) + ' type: ' + str(type(class_color)) + " type: " + str(type(class_color[0])) )
                line_thickness = max(1, math.ceil(max([img_height, img_width])/2000))
                

                success = False
                try:
                    cv2.rectangle(cv2_img_results, bot_left_box, top_right_box, class_color, thickness=line_thickness)
                    success = True
                except Exception as e:
                    self.msg_if.pub_warn("Failed to create bounding box rectangle: " + str(e))

                # Overlay text data on OpenCV image
                if success == True:


                    ## Overlay Text
                    overlay_labels =  self.overlay_labels
                    overlay_range_bearing =  self.overlay_range_bearing

                    overlay_text = ""

                    if overlay_labels:
                        overlay_text = overlay_text + class_name + " "
                    if overlay_range_bearing:
                        rb_text = ''
                        if target_dict['range_m'] != -999 and target_dict['range_m'] != '':
                            rb_text = rb_text + str(round(target_dict['range_m'],1)) + 'm :'
                        if target_dict['azimuth_deg'] != -999 and target_dict['elevation_deg'] != -999:
                            rb_text = rb_text + str(round(target_dict['azimuth_deg'],1)) + 'deg '
                            rb_text = rb_text + str(round(target_dict['elevation_deg'],1)) + 'deg '
                        if len(rb_text) > 0:
                            overlay_text = overlay_text + rb_text



                    if len(overlay_text) > 0:
                        text2overlay=overlay_text
                        text_size = cv2.getTextSize(text2overlay, 
                            font, 
                            fontScale,
                            fontThickness)
                        #self.msg_if.pub_warn("Text Size: " + str(text_size))
                        line_height = text_size[0][1]
                        line_width = text_size[0][0]
                        x_padding = int(line_height*0.4)
                        y_padding = int(line_height*0.4)
                        
                        center = bot_left_box[0] + int(( top_right_box[0] - bot_left_box[0]) / 2 )
                        #bot_left_text = (xmin + (line_thickness * 2) + x_padding , ymin + line_height + (line_thickness * 2) + y_padding)
                        bot_left_text = (center + x_padding , ymin - (line_thickness * 2) - y_padding)
                        # Create Text Background Box
                        #bot_left_box =  (bot_left_text[0] - x_padding , bot_left_text[1] + y_padding)
                        bot_left_box =  ( center - x_padding, bot_left_text[1] + y_padding)
                        top_right_box = (center + line_width + x_padding, bot_left_text[1] - line_height - y_padding )
                        box_color = [0,0,0]

                        try:
                            cv2.rectangle(cv2_img_results, bot_left_box, top_right_box, box_color , -1)
                            cv2.putText(cv2_img_results,text2overlay, 
                                bot_left_text, 
                                font, 
                                fontScale,
                                fontColor,
                                fontThickness,
                                lineType)
                        except Exception as e:
                            self.msg_if.pub_warn("Failed to apply overlay label text: " + str(e))

                        # Start name overlays    
                        x_start = int(img_width * 0.05)
                        y_start = int(img_height * 0.05)
            except:
                pass

        return cv2_img_results


    def targetsResultsCb(self, results_msg, args):
        source_topic = args
        if source_topic != self.selected_image_topic or nepi_sdk.is_shutdown() == True:
            return
        self.targets_results_dict = self.convert_results_pub_msg2dict(results_msg)
        self.targets_time = nepi_utils.get_time()

    def trackResultsCb(self, msg):
        self.track_results = nepi_sdk.convert_msg2dict(msg)
        self.track_results_time = nepi_utils.get_time()

    def statusCb(self, msg):
        self.last_status_time = nepi_utils.get_time()


        self.enabled = True
        self.state_str_msg = ''
        self.max_image_pub_rate_hz = msg.max_image_pub_rate_hz
        self.use_last_image = False #msg.use_last_image
        self.imaging_enabled = True

        last_sel_imgs = copy.deepcopy(self.selected_image_topic)
        self.selected_image_topic = msg.selected_image_topic
        if last_sel_imgs != self.selected_image_topic:
            self.msg_if.pub_info("Updating selected image topic: " + str(self.selected_image_topic))

        self.show_targets_enabled = msg.show_targets_enabled
        self.show_track_enabled = msg.show_track_enabled
        self.show_goal_enabled = msg.show_goal_enabled
        self.goal_error_degs = [msg.auto_pan_error_deg, msg.auto_tilt_error_deg]



    def getImgInfoDict(self):
        self.img_info_lock.acquire()
        img_info_dict = copy.deepcopy(self.img_info_dict)
        self.img_info_lock.release()
        return img_info_dict

    def shutdownCb(self):
        if self.img_info_dict is not None:
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
