/*
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi rui (nepi_rui) repo
# (see https://github.com/nepi-engine/nepi_rui)
#
# License: NEPI RUI repo source-code and NEPI Images that use this source-code
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
 */
import React, { Component } from "react"
import { observer, inject } from "mobx-react"

import Section from "./Section"
import Label from "./Label"
import { Column, Columns } from "./Columns"
import Select, { Option } from "./Select"
import Styles from "./Styles"
import AsyncToggle from "./AsyncToggle"
import BooleanIndicator from "./BooleanIndicator"
import { SliderAdjustment } from "./AdjustmentWidgets"

import NepiIFImageViewer from "./Nepi_IF_ImageViewer"
import NepiIFControls from "./Nepi_IF_Controls"
import NepiIFSaveData from "./Nepi_IF_SaveData"
import NepiIFConfig from "./Nepi_IF_Config"

import { createMenuFirstLastNames } from "./Utilities"

function round(value, decimals = 0) {
  return Number(value).toFixed(decimals)
}

@inject("ros")
@observer

// Obstacles app main panel. Left column is the overlay image viewer, right
// column is the app panel: depth map source selection, process controls, the
// algorithm's ControlsIF box, overlay toggles and status.
//
// This page binds to ONE app node, not to a manager list. The status topic is
// <app>/auto_turret/status carrying nepi_app_auto_turret/ObstaclesStatus, and every
// command topic hangs off the namespace that message reports in
// process_status.namespace -- which is <app>/auto_turret, the namespace
// ObstaclesIF registers its subscribers on. The algorithm's own controls are
// rendered by the shared Nepi_IF_Controls against status_msg.controls_topic.
class NepiAppObstacles extends Component {

  constructor(props) {
    super(props)

    this.state = {
      appName: "app_auto_turret",
      appNamespace: null,

      status_msg: null,
      process_status_msg: null,
      connected: false,

      sources_list_viewable: true,

      selected_display_topic: "None",
      selected_display_text: "None",

      statusListener: null,
      needs_update: false
    }

    this.getBaseNamespace = this.getBaseNamespace.bind(this)
    this.getAppNamespace = this.getAppNamespace.bind(this)
    this.getProcessNamespace = this.getProcessNamespace.bind(this)
    this.getControlsNamespace = this.getControlsNamespace.bind(this)
    this.getSaveNamespace = this.getSaveNamespace.bind(this)

    this.statusListener = this.statusListener.bind(this)
    this.updateStatusListener = this.updateStatusListener.bind(this)

    this.createSourceTopicsOptions = this.createSourceTopicsOptions.bind(this)
    this.toggleSourcesListViewable = this.toggleSourcesListViewable.bind(this)
    this.onSourceTopicSelected = this.onSourceTopicSelected.bind(this)

    this.getDisplayImgOptions = this.getDisplayImgOptions.bind(this)
    this.onDisplayImgSelected = this.onDisplayImgSelected.bind(this)
    this.getSegmentImgTopics = this.getSegmentImgTopics.bind(this)


    this.onPTUpdateText = this.onPTUpdateText.bind(this)
    this.onPTKeyText = this.onPTKeyText.bind(this)

    this.renderApp = this.renderApp.bind(this)
    this.renderAppSettings = this.renderAppSettings.bind(this)
    this.renderPTAuto = this.renderPTAuto.bind(this)
    this.renderPTControls = this.renderPTControls.bind(this)

    this.renderImageViewer = this.renderImageViewer.bind(this)

  }

  getBaseNamespace() {
    const { namespacePrefix, deviceId } = this.props.ros
    var baseNamespace = null
    if (namespacePrefix !== null && deviceId !== null) {
      baseNamespace = "/" + namespacePrefix + "/" + deviceId
    }
    return baseNamespace
  }

  getAppNamespace() {
    const { namespacePrefix, deviceId } = this.props.ros
    var appNamespace = null
    if (namespacePrefix !== null && deviceId !== null) {
      if (this.props.namespace !== undefined) {
        appNamespace = this.props.namespace
      } else {
        appNamespace = "/" + namespacePrefix + "/" + deviceId + "/" + this.state.appName
      }
    }
    return appNamespace
  }

  // Namespace every auto_turret command topic hangs off. Prefer what the node
  // reports so the two can never drift; fall back to the conventional path.
  getProcessNamespace() {
    const appNamespace = this.getAppNamespace()
    return appNamespace
  }

  getControlsNamespace() {
    const status_msg = this.state.status_msg
    if (status_msg != null && status_msg.controls_topic) {
      return status_msg.controls_topic
    }
    const appNamespace = this.getAppNamespace()
    return (appNamespace != null) ? appNamespace + "/controls" : null
  }

  getSaveNamespace() {
    const process_status_msg = this.state.process_status_msg
    if (process_status_msg != null && process_status_msg.save_data_topic) {
      return process_status_msg.save_data_topic
    }
    const appNamespace = this.getAppNamespace()
    return (appNamespace != null) ? appNamespace + "/save_data" : "None"
  }

  // Callback for handling ROS Status messages
  statusListener(message) {
    this.setState({
      status_msg: message,
      process_status_msg: message.process_status,
      connected: true
    })
  }

  // Function for configuring and subscribing to Status
  updateStatusListener(namespace) {
    const statusNamespace = namespace + "/status"
    if (this.state.statusListener) {
      this.state.statusListener.unsubscribe()
    }
    var statusListener = this.props.ros.setupStatusListener(
          statusNamespace,
          "nepi_app_auto_turret/ObstaclesStatus",
          this.statusListener
        )
    this.setState({
      appNamespace: namespace,
      statusListener: statusListener,
    })
  }

  componentDidMount() {
    this.setState({ needs_update: true })
  }

  componentDidUpdate(prevProps, prevState, snapshot) {
    const namespace = this.getAppNamespace()
    const namespace_updated = (this.state.appNamespace !== namespace && namespace !== null)
    if (namespace_updated || this.state.needs_update === true) {
      if (namespace !== null && namespace.indexOf('null') === -1) {
        this.setState({ needs_update: false })
        this.updateStatusListener(namespace)
      }
    }
  }

  componentWillUnmount() {
    if (this.state.statusListener) {
      this.state.statusListener.unsubscribe()
    }
    this.setState({
      status_msg: null,
      process_status_msg: null,
      connected: false,
      statusListener: null,
      selected_display_topic: "None",
      selected_display_text: "None"
    })
  }

  //////////////////////////
  // Source selection

  // Options come from the app's own available_source_topics, which ObstaclesIF
  // fills by discovering DepthMapStatus publishers. The RUI does not do its own
  // topic filtering -- the node is the single source of truth for what this
  // process can consume.
  createSourceTopicsOptions() {
    const process_status_msg = this.state.process_status_msg
    var items = []
    items.push(<Option value={'None'}>{'None'}</Option>)
    if (process_status_msg == null) {
      return items
    }
    const source_options = process_status_msg.available_source_topics
    if (source_options.length === 0) {
      return items
    }
    items.push(<Option value={'All'}>{'All'}</Option>)
    const sourceShortnames = createMenuFirstLastNames(source_options)
    for (var i = 0; i < source_options.length; i++) {
      items.push(<Option value={source_options[i]}>{sourceShortnames[i]}</Option>)
    }
    return items
  }

  toggleSourcesListViewable() {
    const set = !this.state.sources_list_viewable
    this.setState({ sources_list_viewable: set })
  }

  onSourceTopicSelected(event) {
    const { sendStringMsg, sendStringArrayMsg } = this.props.ros
    const process_namespace = this.getProcessNamespace()
    const process_status_msg = this.state.process_status_msg
    if (process_namespace == null || process_status_msg == null) {
      return
    }
    const source_options = process_status_msg.available_source_topics
    const selected_sources = process_status_msg.selected_sources
    const source_topic = event.target.value

    if (source_topic === "None") {
      sendStringArrayMsg(process_namespace + "/remove_source_topics", source_options)
    }
    else if (source_topic === "All") {
      sendStringArrayMsg(process_namespace + "/add_source_topics", source_options)
    }
    else if (selected_sources.indexOf(source_topic) === -1) {
      sendStringMsg(process_namespace + "/add_source_topic", source_topic)
    }
    else {
      sendStringMsg(process_namespace + "/remove_source_topic", source_topic)
    }
  }

  //////////////////////////
  // App panel

  renderApp() {
    const status_msg = this.state.status_msg

    return (
      <Section title={"Obstacles"}>

        <div hidden={(status_msg != null)}>
          <pre style={{ height: "50px", overflowY: "auto" }} align={"left"} textAlign={"left"}>
            {"Loading..."}
          </pre>
        </div>

        {(status_msg != null) ? this.renderAppSettings() : null}

      </Section>
    )
  }

  renderAppSettings() {
    const { sendBoolMsg } = this.props.ros

    const status_msg = this.state.status_msg
    const process_status_msg = this.state.process_status_msg
    const process_namespace = this.getProcessNamespace()
    const controls_namespace = this.getControlsNamespace()

    const enabled = process_status_msg.enabled
    const running = process_status_msg.running
    const processing = process_status_msg.state

    const max_process_rate_hz = process_status_msg.max_process_rate_hz
    const max_image_pub_rate_hz = process_status_msg.max_image_pub_rate_hz

    const imaging_enabled = process_status_msg.image_pub_enabled
    const use_last_image = process_status_msg.use_last_image

    const auto_select_active = process_status_msg.auto_select_active

    const selected_sources = process_status_msg.selected_sources

    const source_selected = process_status_msg.source_selected
    const source_connected = process_status_msg.source_connected

    const avg_process_latency = round(process_status_msg.avg_process_latency, 3)
    const avg_process_rate = round(process_status_msg.avg_process_rate, 3)

    const source_options = this.createSourceTopicsOptions()

    const navpose_connected = status_msg.navpose_topic_connected

    const scanning_ready = status_msg.scanning_ready
    const scanning_enabled = status_msg.scanning_enabled

    const tracking_ready = status_msg.tracking_ready    
    const tracking_enabled = status_msg.tracking_enabled

    const stabilize_ready = status_msg.stabilize_ready
    const stabilize_enabled = status_msg.stabilize_enabled

    const pan_control_disabled = status_msg.pan_control_disabled
    const tilt_control_disabled = status_msg.tilt_control_disabled

    const speed_ratio = status_msg.speed_ratio
    const pan_speed_ratio = status_msg.pan_speed_ratio
    const tilt_speed_ratio = status_msg.tilt_speed_ratio

    const pan_deg = status_msg.pan_deg
    const tilt_deg = status_msg.tilt_deg

    const pan_goal = status_msg.pan_goal
    const tilt_goal = status_msg.tilt_goal

    const pan_deg_per_sec = status_msg.pan_deg_per_sec
    const tilt_deg_per_sec = status_msg.tilt_deg_per_sec

    const pan_goto = status_msg.pan_goto
    const tilt_goto = status_msg.tilt_goto



    return (
      <Columns>
      <Column>

        <Columns>
        <Column>

          <Label title="Auto Select Source">
            <AsyncToggle
              checked={auto_select_active === true}
              onClick={() => sendBoolMsg(process_namespace + "/set_auto_select_enable", !auto_select_active)}>
            </AsyncToggle>
          </Label>



        </Column>
        <Column>

          <div style={{ marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }} />

          <Label title={"Select Pan Tilt"} />
          
          <div onClick={this.toggleSourcesListViewable} style={{ backgroundColor: Styles.vars.colors.grey0 }}>
            <Select style={{ width: "10px" }} />
          </div>
          <div hidden={this.state.sources_list_viewable === false}>
          {source_options.map((source) =>
            <div onClick={this.onSourceTopicSelected}
              style={{
                textAlign: "center",
                padding: `${Styles.vars.spacing.xs}`,
                color: Styles.vars.colors.black,
                backgroundColor: (selected_sources.indexOf(source.props.value) !== -1) ?
                  Styles.vars.colors.blue : Styles.vars.colors.grey0,
                cursor: "pointer",
              }}>
              <body source-topic={source} style={{ color: Styles.vars.colors.black }}>{source}</body>
            </div>
          )}
          </div>

        </Column>
        </Columns>

        <Columns>
        <Column>

          <Label title="Enable Scanning">
            <AsyncToggle
              disabled={scanning_ready == false}
              checked={scanning_enabled === true}
              onClick={() => sendBoolMsg(process_namespace + "/set_scanning_enable", !scanning_enabled)}>
            </AsyncToggle>
          </Label>

          <Label title="Enable Tracking">
            <AsyncToggle
              disabled={tracking_ready == false}
              checked={tracking_enabled === true}
              onClick={() => sendBoolMsg(process_namespace + "/set_tracking_enable", !tracking_enabled)}>
            </AsyncToggle>
          </Label>


          <Label title="Enable Stabilize">
            <AsyncToggle
              disabled={stabilize_ready == false}
              checked={stabilize_enabled === true}
              onClick={() => sendBoolMsg(process_namespace + "/set_stabilize_enable", !stabilize_enabled)}>
            </AsyncToggle>
          </Label>

        </Column>
        <Column>
        </Column>
        </Columns>

        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }} />

        <Label title={"STATUS"}></Label>

        <div style={{ display: 'flex' }}>
          <div style={{ width: '40%' }}>
            <Label title={"Source Selected"}>
              <BooleanIndicator value={source_selected} />
            </Label>
            <Label title={"NavPose"}>
              <BooleanIndicator value={navpose_connected} />
            </Label>
          </div>

          <div style={{ width: '20%' }}>
            {}
          </div>

          <div style={{ width: '40%' }}>
            <Label title={"Source Connected"}>
              <BooleanIndicator value={source_connected} />
            </Label>
          </div>
        </div>

        <div style={{ display: 'flex' }}>
          <div style={{ width: '40%' }}>
            <Label title={"Running"}>
              <BooleanIndicator value={running} />
            </Label>
          </div>

          <div style={{ width: '20%' }}>
            {}
          </div>

          <div style={{ width: '40%' }}>
            <Label title={"Detect State"}>
              <BooleanIndicator value={processing} />
            </Label>
          </div>
        </div>

        <pre style={{ height: "60px" }} align={"left"} textAlign={"left"}>
        {"\n Avg Process Rate: " + avg_process_rate +
         "\n Avg Process Latency: " + avg_process_latency}
        </pre>

        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }} />

        <label style={{ fontWeight: 'bold' }} align={"left"} textAlign={"left"}>
          {"Process Settings"}
        </label>

        <SliderAdjustment
          title={"Max Process Rate"}
          msgType={"std_msgs/Float32"}
          adjustment={max_process_rate_hz}
          topic={process_namespace + "/set_max_process_rate"}
          scaled={1.0}
          min={1}
          max={20}
          disabled={false}
          tooltip={"Sets process max rate in hz"}
          unit={"Hz"}
        />

        <SliderAdjustment
          title={"Max Image Publish Rate"}
          msgType={"std_msgs/Float32"}
          adjustment={max_image_pub_rate_hz}
          topic={process_namespace + "/set_max_image_pub_rate"}
          scaled={1.0}
          min={1}
          max={20}
          disabled={false}
          tooltip={"Sets overlay image max publish rate in hz"}
          unit={"Hz"}
        />



        {(controls_namespace != null) ?
          <NepiIFControls
            namespace={controls_namespace}
            title={"Obstacle Detection Controls"}
          />
        : null}

        <NepiIFConfig
          namespace={this.getAppNamespace()}
          title={"Nepi_IF_Config"}
        />

      </Column>
      </Columns>
    )
  }








  onPTUpdateText(e) {
    var panElement = null
    var tiltElement = null
    if (e.target.id === "PTXPanGoto") 
      {
        panElement = document.getElementById("PTXPanGoto")
        setElementStyleModified(panElement)
        this.setState({panGoto: e.target.value})
             
      }
        
    else if  (e.target.id === "PTXTiltGoto")
        {
          tiltElement = document.getElementById("PTXTiltGoto")
          setElementStyleModified(tiltElement)
          this.setState({tiltGoto: e.target.value})         
          
        }

  }

  onPTKeyText(e) {
    const {ptxDevices, onSetPTXGotoPos, onSetPTXGotoPanPos, onSetPTXGotoTiltPos, onSetPTXHomePos, onSetPTXSoftStopPos} = this.props.ros
 const {sendFloatMsg} = this.props.ros
const namespace = this.getNamespace()
    const selected_pan_tilt = this.state.selected_pan_tilt
    const ptx_caps = ptxDevices[selected_pan_tilt]
    const has_timed_pos = ptx_caps && (ptx_caps.has_timed_positioning)
    const has_sep_pan_tilt = ptx_caps && (ptx_caps.has_seperate_pan_tilt_control)
    const has_abs_pos = ptx_caps && (ptx_caps.has_absolute_positioning === true)
    const has_homing = ptx_caps && (ptx_caps.has_homing)
    const has_speed_control = ptx_caps && (ptx_caps.has_adjustable_speed)
    const has_sep_speed = ptx_caps && (ptx_caps.has_seperate_pan_tilt_speed === true)
    //Unused const has_set_home = ptx_caps && (ptx_caps.has_set_home)

    var panElement = null
    var tiltElement = null
    if(e.key === 'Enter'){
      if (e.target.id === "PTXPanGoto") 
        {
          panElement = document.getElementById("PTXPanGoto")
          tiltElement = document.getElementById("PTXTiltGoto")                    
          sendFloatMsg(namespace + '/set_pan_pos_deg', Number(panElement.value),Number(tiltElement.value))   
          clearElementStyleModified(panElement)   
          this.setState({panGoto: null})    
          
        }
        else if  (e.target.id === "PTXTiltGoto")
          {
            
            panElement = document.getElementById("PTXPanGoto")
            tiltElement = document.getElementById("PTXTiltGoto")


              sendFloatMsg(namespace + '/set_tilt_pos_deg', Number(tiltElement.value))
          
            clearElementStyleModified(tiltElement)
            this.setState({tiltGoto: null})      
          
          }
    }
  }




  renderPTAuto() {
    const { ptxDevices, sendBoolMsg } = this.props.ros

    const { scanPanEnabled, scanTiltEnabled, trackPanEnabled, trackTiltEnabled,
            track_source_connected, stabPanEnabled, stabTiltEnabled,
            click_pan_enabled, click_tilt_enabled  } = this.state /*sinPanEnabled ,sinTiltEnabled*/

    const selected_pan_tilt = this.state.selected_pan_tilt

    //Unused const {sendTriggerMsg} = this.props.ros

    const namespace = this.getNamespace()

    const status_msg = this.state.status_msg
    const topics = Object.keys(ptxDevices)
    const pt_connected_topics = []
    var i
    for (i = 0; i <topics.length; i++) {
    if (topics[i].includes(selected_pan_tilt)){
      pt_connected_topics.push(topics[i])
    }
  }
    
    const pt_connected = (pt_connected_topics.indexOf(selected_pan_tilt) !== -1)
    //console.log('pt_connected: ' + pt_connected)


    if (status_msg == null || pt_connected == false){
      return(

        <Columns>
        <Column>

        </Column>
        </Columns>

      )

    }
    else {


    const has_scan_pan = (status_msg.pt_status_msg.has_scan_pan)
    const has_scan_tilt = (status_msg.pt_status_msg.has_scan_tilt)
    const has_abs_pos = (status_msg.pt_status_msg.has_absolute_positioning)
    const has_homing = (status_msg.pt_status_msg.has_homing)
    const has_speed_control = (status_msg.pt_status_msg.has_adjustable_speed)
    const has_sep_speed = (status_msg.pt_status_msg.has_seperate_pan_tilt_speed)

    const disable_track_enable = ((track_source_connected === false || has_scan_pan === false || has_scan_tilt === false))

    const disable_stab_enable = false


      const scanEnabled = status_msg.scan_enabled
      const trackEnabled = status_msg.track_enabled
      const stabEnabled = status_msg.stab_enabled

      const pan_control_disabled = (status_msg.pan_control_disabled === true)
      const tilt_control_disabled = (status_msg.tilt_control_disabled === true)
      const speedRatio = status_msg.speed_ratio
      const speedPanRatio = status_msg.pan_speed_ratio
      const speedTiltRatio = status_msg.tilt_speed_ratio


      const show_control = this.state.show_control
        return (
          <React.Fragment>



          {/* { (has_homing === false) ?


          <ButtonMenu>
            <Button onClick={() => this.props.ros.sendTriggerMsg(namespace + '/pt_stop')}>{"STOP"}</Button>
          </ButtonMenu>

          :

          <ButtonMenu>
            <Button onClick={() => this.props.ros.sendTriggerMsg(namespace + '/pt_stop')}>{"STOP"}</Button>
            <Button onClick={() => this.props.ros.sendTriggerMsg(namespace + '/pan_home')}>{"PAN HOME"}</Button>
            <Button onClick={() => this.props.ros.sendTriggerMsg(namespace + '/tilt_home')}>{"TILT HOME"}</Button>
          </ButtonMenu>

          } */}




              <Label title={"Stabilize"}>


                <div style={{ display: "inline-block", width: "45%", float: "left" }} >

                      <AsyncToggle style={{justifyContent: "flex-left"}} 
                        disabled={this.state.stabPanReady === false}
                        checked={stabEnabled === true} 
                        onClick={() => sendBoolMsg.bind(this)(namespace + "/set_stab_enable",!stabEnabled)} />
                  
                </div>
{/* 
                <div style={{ display: "inline-block", width: "45%", float: "left" }} >
                  <div hidden={this.state.stabPanReady === false}>
                      <Toggle style={{justifyContent: "flex-left"}} 
                        checked={stabPanEnabled === true} 
                        onClick={() => sendBoolMsg.bind(this)(namespace + "/set_stab_pan_enable",!stabPanEnabled)} />
                  </div>
                </div>


                <div style={{ display: "inline-block", width: "45%", float: "right" }}>
                  <div hidden={this.state.stabTiltReady === false}>
                    <Toggle style={{justifyContent: "flex-right"}} 
                      checked={stabTiltEnabled === true} 
                      onClick={() => sendBoolMsg.bind(this)(namespace + "/set_stab_tilt_enable",!stabTiltEnabled)} />
                  </div>
                </div> */}

              </Label>


            <Label title={"Sweep"}>

              <div style={{ display: "inline-block", width: "45%", float: "left" }}>
                <AsyncToggle style={{justifyContent: "flex-left"}} 
                  checked={scanEnabled} 
                  onClick={() => sendBoolMsg.bind(this)(namespace + "/set_scan_enable",!scanEnabled)} />
              </div>

              {/* <div style={{ display: "inline-block", width: "45%", float: "left" }}>
                <Toggle style={{justifyContent: "flex-left"}} 
                  checked={scanPanEnabled} 
                  onClick={() => sendBoolMsg.bind(this)(namespace + "/set_scan_pan_enable",!scanPanEnabled)} />
              </div>


              <div style={{ display: "inline-block", width: "45%", float: "right" }}>
                <Toggle style={{justifyContent: "flex-right"}} 
                  checked={scanTiltEnabled} 
                  onClick={() => sendBoolMsg.bind(this)(namespace + "/set_scan_tilt_enable",!scanTiltEnabled)} />
              </div> */}

            </Label>


              <Label title={"AI Tracking"}>


                <div style={{ display: "inline-block", width: "45%", float: "left" }}>
                  <AsyncToggle style={{justifyContent: "flex-left"}} 
                    disabled={disable_track_enable === true}
                    checked={trackEnabled === true && disable_track_enable === false} 
                    onClick={() => sendBoolMsg.bind(this)(namespace + "/set_track_enable",!trackEnabled)} />
                </div>

                {/* <div style={{ display: "inline-block", width: "45%", float: "left" }}>
                  <Toggle style={{justifyContent: "flex-left"}} 
                    disabled={disable_track_enable === true}
                    checked={trackPanEnabled === true && disable_track_enable === false} 
                    onClick={() => sendBoolMsg.bind(this)(namespace + "/set_track_pan_enable",!trackPanEnabled)} />
                </div>


                <div style={{ display: "inline-block", width: "45%", float: "right" }}>
                  <Toggle style={{justifyContent: "flex-right"}} 
                    disabled={disable_track_enable === true}
                    checked={trackTiltEnabled === true && disable_track_enable === false} 
                    onClick={() => sendBoolMsg.bind(this)(namespace + "/set_track_tilt_enable",!trackTiltEnabled)} />
                </div> */}

              </Label>



            {this.renderPTControls()}

            </React.Fragment>
        )
  }
}




  renderPTControls() {
    const { ptxDevices} = this.props.ros

    const { scanPanEnabled, scanTiltEnabled, trackPanEnabled, trackTiltEnabled,
            track_source_connected, stabPanEnabled, stabTiltEnabled,
            click_pan_enabled, click_tilt_enabled  } = this.state /*sinPanEnabled ,sinTiltEnabled*/

    const selected_pan_tilt = this.state.selected_pan_tilt

    //Unused const {sendTriggerMsg} = this.props.ros

    const namespace = this.getNamespace()

    const status_msg = this.state.status_msg
    const topics = Object.keys(ptxDevices)
    const pt_connected_topics = []
    var i
    for (i = 0; i <topics.length; i++) {
    if (topics[i].includes(selected_pan_tilt)){
      pt_connected_topics.push(topics[i])
    }
  }
    
    const pt_connected = (pt_connected_topics.indexOf(selected_pan_tilt) !== -1)
    //console.log('pt_connected: ' + pt_connected)


    if (status_msg == null || pt_connected == false){
      return(

        <Columns>
        <Column>

        </Column>
        </Columns>

      )

    }
    else {


    const has_scan_pan = (status_msg.pt_status_msg.has_scan_pan)
    const has_scan_tilt = (status_msg.pt_status_msg.has_scan_tilt)
    const has_abs_pos = (status_msg.pt_status_msg.has_absolute_positioning)
    const has_homing = (status_msg.pt_status_msg.has_homing)
    const has_speed_control = (status_msg.pt_status_msg.has_adjustable_speed)
    const has_sep_speed = (status_msg.pt_status_msg.has_seperate_pan_tilt_speed)

    const disable_track_enable = ((track_source_connected === false || has_scan_pan === false || has_scan_tilt === false))

    const disable_stab_enable = false

      const panPosition = status_msg.pt_status_msg.pan_now_deg
      const tiltPosition = status_msg.pt_status_msg.tilt_now_deg

      const panPositionClean = panPosition + .001
      const tiltPositionClean = tiltPosition + .001

      if (this.state.panGoto == null){
        this.setState({panGoto: panPositionClean})
      }

      if (this.state.tiltGoto == null){
        this.setState({tiltGoto: tiltPositionClean})
      }

      const panMove = status_msg.pt_status_msg.pan_goal_deg
      const tiltMove = status_msg.pt_status_msg.tilt_goal_deg

      const panMoveClean = panMove + .001
      const tiltMoveClean = tiltMove + .001

      const pan_control_disabled = (status_msg.auto_pan_pos_disabled === true)
      
      if (pan_control_disabled !== this.state.panDisabled){
        this.setState({panGoto: panMoveClean, panDisabled: pan_control_disabled})
      }
      const pan_pos = this.state.panGoto //pan_control_disabled === true ? panMoveClean : this.state.panGoto

      const tilt_control_disabled = (status_msg.auto_tilt_pos_disabled === true)
      if (tilt_control_disabled !== this.state.tiltDisabled){
        this.setState({tiltGoto: tiltMoveClean, tiltDisabled: tilt_control_disabled})
      }      
      const tilt_pos = this.state.tiltGoto //tilt_control_disabled === true ? tiltMoveClean : this.state.tiltGoto


      const panCurSpeed = status_msg.pt_status_msg.speed_pan_dps
      const tiltCurSpeed = status_msg.pt_status_msg.speed_tilt_dps

      const panCurSpeedClean = panCurSpeed + .001
      const tiltCurSpeedClean = tiltCurSpeed + .001

      const speedRatio = status_msg.speed_ratio
      const speedPanRatio = status_msg.auto_pan_speed_ratio_set
      const speedTiltRatio = status_msg.auto_tilt_speed_ratio_set

      const maxSpeed = status_msg.pan_tilt_max_speed_dps
      const panSetSpeed = speedPanRatio * maxSpeed
      const tiltSetSpeed = speedTiltRatio * maxSpeed

      const panSetSpeedClean = panSetSpeed + .001
      const tiltSetSpeedClean = tiltSetSpeed + .001

        return (
          <React.Fragment>

          <Label title={""} style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
            <div style={{ display: "inline-block", width: "45%", float: "left" }}>{"Pan"}</div>
            <div style={{ display: "inline-block", width: "45%", float: "left" }}>{"Tilt"}</div>
          </Label>

          <ButtonMenu>
            <Button onClick={() => this.props.ros.sendTriggerMsg(namespace + '/pt_stop')}>{"STOP"}</Button>
            <Button onClick={() => this.props.ros.sendTriggerMsg(namespace + '/pan_home')}>{"HOME"}</Button>
            <Button onClick={() => this.props.ros.sendTriggerMsg(namespace + '/tilt_home')}>{"HOME"}</Button>
          </ButtonMenu>

          <div hidden={(has_abs_pos === false)}>

{/* 
              <Label title={"GoTo Position "}>
                <Input
                  disabled={pan_control_disabled === true}
                  id={"PTXPanGoto"}
                  style={{ width: "45%", float: "left" }}
                  value={round(pan_pos,1)}
                  onChange= {this.onPTUpdateText}
                  onKeyDown= {this.onPTKeyText}
                />
                <Input
                  disabled={tilt_control_disabled === true}
                  id={"PTXTiltGoto"}
                  style={{ width: "45%" }}
                  value={round(tilt_pos,1)}
                  onChange= {this.onPTUpdateText}
                  onKeyDown= {this.onPTKeyText}
                />
              </Label> */}


              <Label title={"Current Position"}>
                <Input
                  disabled={true}
                  style={{ width: "45%", float: "left" }}
                  value={round(panPositionClean, 1)}
                />
                <Input
                  disabled={true}
                  style={{ width: "45%" }}
                  value={round(tiltPositionClean, 1)}
                />
              </Label>

              <Label title={"Average Speed"}>
                <Input
                  disabled={true}
                  style={{ width: "45%", float: "left" }}
                  value={round(panCurSpeedClean, 1)}
                />
                <Input
                  disabled={true}
                  style={{ width: "45%" }}
                  value={round(tiltCurSpeedClean, 1)}
                />
              </Label>

  
          </div>



          <div hidden={(has_speed_control === false)}>

              <React.Fragment>
                <SliderAdjustment
                  disabled={status_msg.auto_pan_position_disabled}
                  title={"Pan Speed"}
                  msgType={"std_msgs/Float32"}
                  adjustment={speedPanRatio}
                  topic={namespace + "/set_pan_speed_ratio"}
                  scaled={0.01}
                  min={0}
                  max={100}
                  tooltip={"Speed as a percentage (0%=min, 100%=max)"}
                  displayValue={round(panSetSpeedClean,1)}
                  unit={""}
                />
                <SliderAdjustment
                  disabled={status_msg.auto_tilt_position_disabled}
                  title={"Tilt Speed"}
                  msgType={"std_msgs/Float32"}
                  adjustment={speedTiltRatio}
                  topic={namespace + "/set_tilt_speed_ratio"}
                  scaled={0.01}
                  min={0}
                  max={100}
                  tooltip={"Speed as a percentage (0%=min, 100%=max)"}
                  displayValue={round(tiltSetSpeedClean,1)}
                  unit={""}
                />
              </React.Fragment>
  
          </div>


            </React.Fragment>
        )
  }
}















  //////////////////////////
  // Image viewer

  // The overlay image topics are reported by the node in
  // process_status.imaging_pub_topics, one per active source.
  getDisplayImgOptions() {
    const { imageTopics } = this.props.ros
    var items = []
    const process_status_msg = this.state.process_status_msg

    var selected_image_topic = this.state.selected_display_topic
    const selected_image_topic_found = (imageTopics.indexOf(selected_image_topic)) !== -1

    if (process_status_msg == null) {
      items.push(<Option value={"None"}>{"None"}</Option>)
      return items
    }

    const image_pub_topics = process_status_msg.imaging_pub_topics
    const image_names = createMenuFirstLastNames(image_pub_topics)
    if (image_pub_topics.length === 0) {
      items.push(<Option value={"None"}>{"None"}</Option>)
      if (selected_image_topic !== 'None') {
        this.setState({ selected_display_topic: "None", selected_display_text: "None" })
      }
      return items
    }

    if (selected_image_topic_found === false) {
      selected_image_topic = image_pub_topics[0]
      if (imageTopics.indexOf(selected_image_topic) !== -1) {
        this.setState({ selected_display_topic: selected_image_topic, selected_display_text: image_names[0] })
      }
    }
    for (var i = 0; i < image_pub_topics.length; i++) {
      if (imageTopics.indexOf(image_pub_topics[i]) !== -1) {
        items.push(<Option value={image_pub_topics[i]}>{image_names[i]}</Option>)
        if ((selected_image_topic === "None" || selected_image_topic === '') && i === 0) {
          this.setState({ selected_display_topic: image_pub_topics[i], selected_display_text: image_names[i] })
        }
      }
    }
    return items
  }

  onDisplayImgSelected(event) {
    const source_topic = event.target.value
    const names = createMenuFirstLastNames([source_topic])
    this.setState({
      selected_display_topic: source_topic,
      selected_display_text: names[0]
    })
  }



  renderImageViewer() {
    const {sendBoolMsg } = this.props.ros
    const selected_image_topic_topic = this.state.selected_display_topic
    const img_publishing = imageTopics.indexOf(selected_image_topic_topic) !== -1

    const selected_image_topic = (img_publishing === true && this.state.connected === true) ? selected_image_topic_topic : "None"
    const selected_image_topic_text = (selected_image_topic_topic === 'None') ? 'No Image Selected' :
      img_publishing ? this.state.selected_display_text : 'Waiting for image to publish'


    const full_screen_enabled = status_msg.full_screen_enabled
    const show_targets_enabled = status_msg.show_targets_enabled
    const show_track_enabled = status_msg.show_track_enabled
    const show_crosshair_enabled = status_msg.show_crosshair_enabled
    return (
           <Section>


                <div style={{ display: 'flex' }}>

 

                      <div style={{ width: '10%' }} centered={"true"}>

                          <Label title="Full Screen">
                            <AsyncToggle
                              checked={full_screen_enabled === true}
                              onClick={() => sendBoolMsg(process_namespace + "/set_full_screen", full_screen_enabled === false)}>
                            </AsyncToggle>
                          </Label>
                      </div>

   
                    <div style={{ width: '5%' }} centered={"true"} >
                        {null}
                      </div>


                      <div style={{ width: '10%' }} centered={"true"}>



                          <Label title="Show Targets">
                            <AsyncToggle
                              checked={show_targets_enabled === true}
                              onClick={() => sendBoolMsg(process_namespace + "/set_show_targets", show_targets_enabled === false)}>
                            </AsyncToggle>
                          </Label>


                      </div>

                      <div style={{ width: '5%' }} centered={"true"} >
                        {null}
                      </div>

                      <div style={{ width: '10%' }} centered={"true"}>

                        <Label title="Show Track">
                          <AsyncToggle
                            checked={show_targets_enabled === true}
                            onClick={() => sendBoolMsg(process_namespace + "/set_show_track", show_track_enabled === false)}>
                          </AsyncToggle>
                        </Label>

                      </div>


                   <div style={{ width: '5%' }} centered={"true"} >
                        {null}
                      </div>


                      <div style={{ width: '10%' }} centered={"true"}>

            <Label title="Show Crosshair">
              <AsyncToggle
                checked={show_crosshair_enabled === true}
                onClick={() => sendBoolMsg(process_namespace + "/set_show_crosshair", show_crosshair_enabled === false)}>
              </AsyncToggle>
            </Label>


                      </div>


                      <div style={{ width: '5%' }} centered={"true"} >
                        {null}
                      </div>


  
                      <div style={{ width: '10%' }} centered={"true"}  hidden={image_stab_supported === false}>

                          {/* <Label title={'Image Stab'}>
                              <AsyncToggle style={{justifyContent: "flex-right"}} 
                                checked={image_stab_enabled === true} 
                                onClick={() => sendBoolMsg.bind(this)(image_stab_update_topic,!image_stab_enabled)} />
                            </Label> */}


                      </div>


                      <div style={{ width: '5%' }} centered={"true"} >
                        {null}
                      </div>

                      <div style={{ width: '10%' }} centered={"true"} >

                          <Label title={'Full Screen'}>
                              <AsyncToggle style={{justifyContent: "flex-right"}} 
                                checked={full_screen_enabled === true} 
                                onClick={() => sendBoolMsg.bind(this)(full_screen_update_topic,!full_screen_enabled)} />
                            </Label>


                      </div>


                      <div style={{ width: '5%' }} centered={"true"} >
                        {null}
                      </div>


                      <div style={{ width: '10%' }} centered={"true"} hidden={show_selectors_option === false}>


                    </div>


                </div>


        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }} />
                  

        <NepiIFImageViewer
          image_topic={selected_image_topic}
          title={selected_image_topic_text}
          show_res_orient={false}
          make_section={false}
          save_data_topic={save_data_topic}
        />


            </Section>
    )

  }




  render() {
    const status_msg = this.state.status_msg
    const save_data_topic = this.getSaveNamespace()

    return (
      <Columns>
      <Column equalWidth={false}>

        {this.renderImageViewer()}

        {(save_data_topic !== 'None' && this.state.connected === true) ?
          <NepiIFSaveData
            saveNamespace={save_data_topic}
            title={"Nepi_IF_SaveData"}
          />
        : null}



        {this.renderApp()}

      </Column>
      </Columns>
    )
  }

}

export default NepiAppObstacles
