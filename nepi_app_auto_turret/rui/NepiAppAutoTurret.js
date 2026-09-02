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

import Toggle from "react-toggle"
import Section from "./Section"
import Label from "./Label"
import { Column, Columns } from "./Columns"
import Select, { Option } from "./Select"
import Styles from "./Styles"
import Input from "./Input"
import Button, { ButtonMenu } from "./Button"
import AsyncToggle from "./AsyncToggle"
import BooleanIndicator from "./BooleanIndicator"
import { SliderAdjustment } from "./AdjustmentWidgets"

import NepiIFImageViewer from "./Nepi_IF_ImageViewer"
import NepiIFConnectPTX from "./Nepi_IF_ConnectPTX"
import NepiIFConnectData from "./Nepi_IF_ConnectData"
import NepiIFConnectTargets from "./Nepi_IF_ConnectTargets"
import NepiIFConnectNavPose from "./Nepi_IF_ConnectNavPose"
import NepiIFSaveData from "./Nepi_IF_SaveData"
import NepiIFConfig from "./Nepi_IF_Config"

import Nepi_IF_ConnectProcess from "./Nepi_IF_ConnectProcess"

import { createMenuFirstLastName, createMenuFirstLastNames, onChangeChangeStateValue } from "./Utilities"
import { setElementStyleModified, clearElementStyleModified } from "./Utilities"

function round(value, decimals = 0) {
  return Number(value).toFixed(decimals)
}

@inject("ros")
@observer

// Auto Turret app main panel. Left column (75%) is the overlay image viewer with
// its four display toggles and the save-data block; right column (23%) is the app
// panel: source selection, scan/track/stabilize enables, status and settings.
// Full screen collapses to the viewer alone.
//
// This page binds to ONE app node, not to a manager list. The status topic is
// <app>/status carrying nepi_app_auto_turret/AutoTurretStatus, and every command
// topic hangs off the app namespace. The algorithm's own controls are rendered by
// the shared Nepi_IF_ConnectProcess against status_msg.controls_topic -- a field
// AutoTurretStatus does not yet define, so that block falls back to <app>/controls
// and stays empty until either the field or a ControlsIF is added.
class NepiAppAutoTurret extends Component {

  constructor(props) {
    super(props)

    this.state = {
      appName: "app_auto_turret",
      appNamespace: null,

      status_msg: null,
      connected: false,

      sources_list_viewable: true,

      selected_display_topic: "None",
      selected_display_text: "None",

      // Manual GoTo entry values and the auto-mode ownership they were seeded
      // against; maintained by statusListener, read by renderPTControls.
      panGoto: null,
      lastPanGoto: null,
      tiltGoto: null,
      lastTiltGoto: null,

      show_control: 'None',

      statusListener: null,
      needs_update: false
    }

    this.getBaseNamespace = this.getBaseNamespace.bind(this)
    this.getAppNamespace = this.getAppNamespace.bind(this)
    this.getSaveNamespace = this.getSaveNamespace.bind(this)

    this.statusListener = this.statusListener.bind(this)
    this.updateStatusListener = this.updateStatusListener.bind(this)


    this.onPTUpdateText = this.onPTUpdateText.bind(this)
    this.onPTKeyText = this.onPTKeyText.bind(this)
    this.onStopClick = this.onStopClick.bind(this)
    this.renderApp = this.renderApp.bind(this)
    this.renderAutoSettings = this.renderAutoSettings.bind(this)
    this.renderPTButtons = this.renderPTButtons.bind(this)
    this.renderPTControls = this.renderPTControls.bind(this)
    this.renderImageViewer = this.renderImageViewer.bind(this)
    this.rendeAutoControls = this.rendeAutoControls.bind(this)

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



  getSaveNamespace() {
    const status_msg = this.state.status_msg
    if (status_msg != null && status_msg.save_data_topic) {
      return status_msg.save_data_topic
    }
    const appNamespace = this.getAppNamespace()
    return (appNamespace != null) ? appNamespace + "/save_data" : "None"
  }

  // Callback for handling ROS Status messages
  statusListener(message) {
    // Seed the manual GoTo entries from the device's reported goal, and reseed
    // whenever an auto mode takes or releases an axis so a stale hand-typed
    // value is never left in the box. This belongs here and not in
    // renderPTControls: a setState from inside render loops.
    const pantilt_status_msg = message.pantilt_status_msg

    const pan_disabled = (message.pan_control_manaul_enabled === true)
    const pan_goal_deg = round((message.pan_control_manaul_enabled === true) ? pantilt_status_msg.pan_goal_deg : message.auto_pan_goal_deg, 1)
    if (this.state.panGoto == null || pan_goal_deg !== this.state.lastPanGoto) {
      this.setState({ panGoto: pan_goal_deg, lastPanGoto: pan_goal_deg})
    }

    const tilt_disabled = (message.tilt_control_disabled === true)
    const tilt_goal_deg = round((message.tilt_control_manaul_enabled === true) ? pantilt_status_msg.tilt_goal_deg : message.auto_tilt_goal_deg, 1)
    if (this.state.tiltGoto == null || tilt_goal_deg !== this.state.lastTiltGoto) {
      this.setState({ tiltGoto: tilt_goal_deg, lastTiltGoto: tilt_goal_deg})
    }
    
    this.setState({
      status_msg: message,
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
          "nepi_app_auto_turret/AutoTurretStatus",
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
    const app_namespace = this.getAppNamespace()
    const namespace_updated = (this.state.appNamespace !== app_namespace && app_namespace !== null)
    if (namespace_updated || this.state.needs_update === true) {
      if (app_namespace !== null && app_namespace.indexOf('null') === -1) {
        this.setState({ needs_update: false })
        this.updateStatusListener(app_namespace)
      }
    }
  }

  componentWillUnmount() {
    if (this.state.statusListener) {
      this.state.statusListener.unsubscribe()
    }
    this.setState({
      status_msg: null,
      connected: false,
      statusListener: null,
      selected_display_topic: "None",
      selected_display_text: "None"
    })
  }



  //////////////////////////
  // App panel

  renderApp() {
    const status_msg = this.state.status_msg

    return (
      <Section title={"Auto Turret"}>

        <div hidden={(status_msg != null)}>
          <pre style={{ height: "50px", overflowY: "auto" }} align={"left"} textAlign={"left"}>
            {"Loading..."}
          </pre>
        </div>

        {(status_msg != null) ? this.renderAutoSettings() : null}

      </Section>
    )
  }

  renderAutoSettings() {
    const { sendBoolMsg } = this.props.ros

    const status_msg = this.state.status_msg

    const app_namespace = this.getAppNamespace()



    const pantilt_connected = status_msg.pantilt_connected
    

    const scanning_ready = status_msg.scanning_ready
    const scanning_enabled = status_msg.scanning_enabled

    const tracking_ready = status_msg.tracking_ready    
    const tracking_enabled = status_msg.tracking_enabled

    const stabilize_ready = status_msg.stabilize_ready
    const stabilize_enabled = status_msg.stabilize_enabled



    return (
      <Columns>
      <Column>

        <div hidden={pantilt_connected === false}>


          <Columns>
          <Column>

            <Label title="Enable Scanning">
              <AsyncToggle
                disabled={scanning_ready == false}
                checked={scanning_enabled === true}
                onClick={() => sendBoolMsg(app_namespace + "/set_scanning_enable", !scanning_enabled)}>
              </AsyncToggle>
            </Label>

            <Label title="Enable Tracking">
              <AsyncToggle
                disabled={tracking_ready == false}
                checked={tracking_enabled === true}
                onClick={() => sendBoolMsg(app_namespace + "/set_tracking_enable", !tracking_enabled)}>
              </AsyncToggle>
            </Label>


            <Label title="Enable Stabilize">
              <AsyncToggle
                disabled={stabilize_ready == false}
                checked={stabilize_enabled === true}
                onClick={() => sendBoolMsg(app_namespace + "/set_stabilize_enable", !stabilize_enabled)}>
              </AsyncToggle>
            </Label>

          </Column>
          <Column>

          {this.renderPTButtons()}

          </Column>
          </Columns>

          
          {this.renderPTControls()}

          </div>


          {this.rendeAutoControls()}

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

  // Drives the GoTo Position inputs in renderPTControls, which are commented
  // out. Kept working so re-enabling them is a one-block change: each axis
  // publishes its own degree command to the app, which gates and forwards.
  onPTKeyText(e) {
    const { sendFloatMsg } = this.props.ros
    const app_namespace = this.getAppNamespace()
    if (app_namespace == null) {
      return
    }

    const status_msg = this.state.status_msg
    const pantilt_namespace = status_msg.selected_pantilt_topic

    const pan_control_manaul_enabled = status_msg.pan_control_manaul_enabled
    const pan_control_namespace = (pan_control_manaul_enabled === true) ? pantilt_namespace : app_namespace
    const pan_control_pos_namespace = (pan_control_manaul_enabled === true) ? pantilt_namespace + '/goto_pan_position'  : app_namespace + '/set_pan_pos_deg'

    const tilt_control_manaul_enabled = status_msg.tilt_control_manaul_enabled
    const tilt_control_namespace = (tilt_control_manaul_enabled === true) ? pantilt_namespace : app_namespace
    const tilt_control_pos_namespace = (tilt_control_manaul_enabled === true) ? pantilt_namespace + '/goto_tilt_position'  : app_namespace + '/set_tilt_pos_deg'

    var panElement = null
    var tiltElement = null
    if(e.key === 'Enter'){
      if (e.target.id === "PTXPanGoto")
        {
          panElement = document.getElementById("PTXPanGoto")
          sendFloatMsg(pan_control_pos_namespace, Number(panElement.value))
          clearElementStyleModified(panElement)
          this.setState({panGoto: null})

        }
        else if  (e.target.id === "PTXTiltGoto")
          {
            tiltElement = document.getElementById("PTXTiltGoto")
            sendFloatMsg(tilt_control_pos_namespace, Number(tiltElement.value))
            clearElementStyleModified(tiltElement)
            this.setState({tiltGoto: null})

          }
    }
  }





  onStopClick(app_namespace, pantilt_namespace) {
      this.props.ros.sendTriggerMsg(app_namespace + '/pt_stop')
      this.props.ros.sendTriggerMsg(pantilt_namespace + '/stop_moving')

  }

  // Condensed pan tilt readout and speed controls for the connected device.
  // Every field below is read off the app's own status: the device dashboard
  // fields come through status_msg.pantilt_status_msg, which the node fills
  // from the connector, and the ratios the app itself owns come off the top
  // level. Nothing here reads the ros store's ptxDevices.
  renderPTButtons() {
    const status_msg = this.state.status_msg
    const app_namespace = this.getAppNamespace()
    const pantilt_connected = (status_msg != null) ? status_msg.pantilt_connected : false
    if (status_msg == null || status_msg.pantilt_connected !== true || app_namespace == null){
      return(

        <Columns>
        <Column>

        </Column>
        </Columns>

      )

    }
    else {
      const { onPTXJogPan, onPTXJogTilt, onPTXJogSpeedPan, onPTXJogSpeedTilt, onPTXStop, onPTXPanStop, onPTXTiltStop } = this.props.ros
      const pantilt_namespace = status_msg.selected_pantilt_topic
      const pantilt_status_msg = status_msg.pantilt_status_msg
      const has_abs_pos = (pantilt_status_msg.has_absolute_positioning === true)
      const has_homing = (pantilt_status_msg.has_homing === true)
      const has_seperate_pan_tilt_control = (pantilt_status_msg.has_seperate_pan_tilt_control === true)
      const has_seperate_pan_tilt_speed = (pantilt_status_msg.has_seperate_pan_tilt_control === true)
      const has_speed_control = (pantilt_status_msg.has_adjustable_speed === true)

      const pan_control_disabled = status_msg.pan_control_disabled === true || pantilt_connected === false
      const pan_control_manaul_enabled = status_msg.pan_control_manaul_enabled
      const pan_control_auto_enabled = status_msg.pan_control_auto_enabled
      const pan_control_namespace = (pan_control_manaul_enabled === true) ? pantilt_namespace : app_namespace


      const tilt_control_disabled = status_msg.tilt_control_disabled === true || pantilt_connected === false
      const tilt_control_manaul_enabled = status_msg.tilt_control_manaul_enabled
      const tilt_control_auto_enabled = status_msg.tilt_control_auto_enabled
      const tilt_control_namespace = (tilt_control_manaul_enabled === true) ? pantilt_namespace : app_namespace

      const panPositionClean = pantilt_status_msg.pan_now_deg + .001
      const tiltPositionClean = pantilt_status_msg.tilt_now_deg + .001

      const panCurSpeedClean = pantilt_status_msg.speed_pan_dps + .001
      const tiltCurSpeedClean = pantilt_status_msg.speed_tilt_dps + .001

      // The app's own stored ratios, which it pushes to the device on connect.
      const speedPanRatio = pantilt_status_msg.speed_pan_ratio
      const speedTiltRatio = pantilt_status_msg.speed_tilt_ratio
      const speedPanTiltRatio = pantilt_status_msg.speed_ratio

      // pan_tilt_max_speed_dps is UNSET_VALUE (-999) until the app has a
      // connected device to ask, which would render a negative dps readout;
      // fall back to what the device reports for itself.
      const maxSpeed = pantilt_status_msg.speed_max_dps
      const panSetSpeed = speedPanRatio * maxSpeed
      const tiltSetSpeed = speedTiltRatio * maxSpeed

      const panSetSpeedClean = panSetSpeed + .001
      const tiltSetSpeedClean = tiltSetSpeed + .001


        // Editable values for the (commented out) GoTo inputs. They live in state
        // so typing can hold; statusListener seeds and reseeds them, because a
        // setState from inside render loops.
        const pan_pos = this.state.panGoto
        const tilt_pos = this.state.tiltGoto

        return (
          <React.Fragment>



          {/* STOP always; the two HOME buttons only for a device that homes.
              All three are Empty triggers on the app, matching pt_stop /
              pan_home / tilt_home in the node's subscriber dict. */}
          {(has_homing === true && has_seperate_pan_tilt_control === true) ?
            <ButtonMenu>
              <Button onClick={() => this.onStopClick(app_namespace,pantilt_namespace)}>{"STOP"}</Button>
              <Button disabled={pan_control_manaul_enabled === false} onClick={() => this.props.ros.sendTriggerMsg(pan_control_namespace + '/pan_home')}>{"P-HOME"}</Button>
              <Button disabled={tilt_control_manaul_enabled === false} onClick={() => this.props.ros.sendTriggerMsg(tilt_control_namespace + '/tilt_home')}>{"T-HOME"}</Button>
            </ButtonMenu>
                  : (has_homing === true) ?

                      <ButtonMenu>
                      <Button onClick={() => this.onStopClick(app_namespace,pantilt_namespace)}>{"STOP"}</Button>
                      <Button disabled={pan_control_manaul_enabled === false || tilt_control_manaul_enabled === false} onClick={() => this.props.ros.sendTriggerMsg(pan_control_namespace + '/go_home')}>{"HOME"}</Button>
                    </ButtonMenu>
                    :
                        <ButtonMenu>
                          <Button onClick={() => this.onStopClick(app_namespace,pantilt_namespace)}>{"STOP"}</Button>
                        </ButtonMenu>
          }




                          <ButtonMenu>

                            <Button
                              disabled={pan_control_manaul_enabled === false} 
                              buttonDownAction={() => onPTXJogPan(pantilt_namespace,  1)}
                              buttonUpAction={() => onPTXPanStop(pantilt_namespace)}>
                              {'\u25C0'}
                            </Button>
                            <Button
                              disabled={pan_control_manaul_enabled === false} 
                              buttonDownAction={() => onPTXJogPan(pantilt_namespace, -1)}
                              buttonUpAction={() => onPTXPanStop(pantilt_namespace)}>
                              {'\u25B6'}
                            </Button>
                            <Button
                              disabled={tilt_control_manaul_enabled === false}
                              buttonDownAction={() => onPTXJogTilt(pantilt_namespace, -1)}
                              buttonUpAction={() => onPTXTiltStop(pantilt_namespace)}>
                              {'\u25B2'}
                            </Button>
                            <Button
                              disabled={tilt_control_manaul_enabled === false}
                              buttonDownAction={() => onPTXJogTilt(pantilt_namespace,  1)}
                              buttonUpAction={() => onPTXTiltStop(pantilt_namespace)}>
                              {'\u25BC'}
                            </Button>


                          </ButtonMenu>
           

            </React.Fragment>
        )
  }
}




  renderPTControls() {
    const status_msg = this.state.status_msg
    const app_namespace = this.getAppNamespace()
    const pantilt_connected = (status_msg != null) ? status_msg.pantilt_connected : false
    if (status_msg == null || status_msg.pantilt_connected !== true || app_namespace == null){
      return(

        <Columns>
        <Column>

        </Column>
        </Columns>

      )

    }
    else {
      const { onPTXJogPan, onPTXJogTilt, onPTXJogSpeedPan, onPTXJogSpeedTilt, onPTXStop, onPTXPanStop, onPTXTiltStop } = this.props.ros
      const pantilt_namespace = status_msg.selected_pantilt_topic
      const pantilt_status_msg = status_msg.pantilt_status_msg
      const has_abs_pos = (pantilt_status_msg.has_absolute_positioning === true)
      const has_homing = (pantilt_status_msg.has_homing === true)
      const has_seperate_pan_tilt_control = (pantilt_status_msg.has_seperate_pan_tilt_control === true)
      const has_seperate_pan_tilt_speed = (pantilt_status_msg.has_seperate_pan_tilt_control === true)
      const has_speed_control = (pantilt_status_msg.has_adjustable_speed === true)

      const pan_control_disabled = status_msg.pan_control_disabled === true || pantilt_connected === false
      const pan_control_manaul_enabled = status_msg.pan_control_manaul_enabled
      const pan_control_auto_enabled = status_msg.pan_control_auto_enabled
      const pan_control_namespace = (pan_control_manaul_enabled === true) ? pantilt_namespace : app_namespace


      const tilt_control_disabled = status_msg.tilt_control_disabled === true || pantilt_connected === false
      const tilt_control_manaul_enabled = status_msg.tilt_control_manaul_enabled
      const tilt_control_auto_enabled = status_msg.tilt_control_auto_enabled
      const tilt_control_namespace = (tilt_control_manaul_enabled === true) ? pantilt_namespace : app_namespace

      const panPositionClean = pantilt_status_msg.pan_now_deg + .001
      const tiltPositionClean = pantilt_status_msg.tilt_now_deg + .001

      const panCurSpeedClean = pantilt_status_msg.speed_pan_dps + .001
      const tiltCurSpeedClean = pantilt_status_msg.speed_tilt_dps + .001

      // The app's own stored ratios, which it pushes to the device on connect.
      const speedPanRatio = pantilt_status_msg.speed_pan_ratio
      const speedTiltRatio = pantilt_status_msg.speed_tilt_ratio
      const speedPanTiltRatio = pantilt_status_msg.speed_ratio

      // pan_tilt_max_speed_dps is UNSET_VALUE (-999) until the app has a
      // connected device to ask, which would render a negative dps readout;
      // fall back to what the device reports for itself.
      const maxSpeed = pantilt_status_msg.speed_max_dps
      const panSetSpeed = speedPanRatio * maxSpeed
      const tiltSetSpeed = speedTiltRatio * maxSpeed

      const panSetSpeedClean = panSetSpeed + .001
      const tiltSetSpeedClean = tiltSetSpeed + .001


        // Editable values for the (commented out) GoTo inputs. They live in state
        // so typing can hold; statusListener seeds and reseeds them, because a
        // setState from inside render loops.
        const pan_pos = this.state.panGoto
        const tilt_pos = this.state.tiltGoto

        return (
          <React.Fragment>
           

          <Label title={""} style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
            <div style={{ display: "inline-block", width: "45%", float: "left" }}>{"Pan"}</div>
            <div style={{ display: "inline-block", width: "45%", float: "left" }}>{"Tilt"}</div>
          </Label>

          <div hidden={(has_abs_pos === false)}>


              <Label title={"GoTo Position "}>
                <Input
                  disabled={pan_control_disabled === true}
                  id={"PTXPanGoto"}
                  style={{ width: "45%", float: "left" }}
                  value={pan_pos}
                  onChange= {this.onPTUpdateText}
                  onKeyDown= {this.onPTKeyText}
                />
                <Input
                  disabled={tilt_control_disabled === true}
                  id={"PTXTiltGoto"}
                  style={{ width: "45%" }}
                  value={tilt_pos}
                  onChange= {this.onPTUpdateText}
                  onKeyDown= {this.onPTKeyText}
                />
              </Label>


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



          <div hidden={(has_speed_control === false || has_seperate_pan_tilt_speed === false)}>

              {/* Speed is never gated on an auto mode: the app stores both
                  ratios as params and re-pushes them to the device on connect,
                  so setting speed while scanning or tracking is legitimate and
                  the node's setPanSpeedRatioCb accepts it. (The source app
                  gated these on auto_pan_position_disabled, a field that does
                  not exist in AutoTurretStatus, so the guard read undefined and
                  never fired anyway.) */}
              <React.Fragment>
                <SliderAdjustment
                  disabled={pan_control_manaul_enabled === false}
                  title={"Pan Speed"}
                  msgType={"std_msgs/Float32"}
                  adjustment={speedPanRatio}
                  topic={pantilt_namespace + "/set_pan_speed_ratio"}
                  scaled={0.01}
                  min={0}
                  max={100}
                  tooltip={"Speed as a percentage (0%=min, 100%=max)"}
                  displayValue={round(panSetSpeedClean,1)}
                  unit={""}
                />
                <SliderAdjustment
                  disabled={tilt_control_manaul_enabled === false}
                  title={"Tilt Speed"}
                  msgType={"std_msgs/Float32"}
                  adjustment={speedTiltRatio}
                  topic={pantilt_namespace + "/set_tilt_speed_ratio"}
                  scaled={0.01}
                  min={0}
                  max={100}
                  tooltip={"Speed as a percentage (0%=min, 100%=max)"}
                  displayValue={round(tiltSetSpeedClean,1)}
                  unit={""}
                />
              </React.Fragment>
  
          </div>

 <div hidden={(has_speed_control === false || has_seperate_pan_tilt_speed === true)}>

              {/* Speed is never gated on an auto mode: the app stores both
                  ratios as params and re-pushes them to the device on connect,
                  so setting speed while scanning or tracking is legitimate and
                  the node's setPanSpeedRatioCb accepts it. (The source app
                  gated these on auto_pan_position_disabled, a field that does
                  not exist in AutoTurretStatus, so the guard read undefined and
                  never fired anyway.) */}
              <React.Fragment>
                <SliderAdjustment
                  disabled={pan_control_manaul_enabled === false}
                  title={"PanTilt Speed"}
                  msgType={"std_msgs/Float32"}
                  adjustment={speedPanTiltRatio}
                  topic={pantilt_namespace + "/set_speed_ratio"}
                  scaled={0.01}
                  min={0}
                  max={100}
                  tooltip={"Speed as a percentage (0%=min, 100%=max)"}
                  displayValue={round(panSetSpeedClean,1)}
                  unit={""}
                />
             
              </React.Fragment>
  
          </div>



            </React.Fragment>
        )
  }
}












  renderImageViewer() {
    const { sendBoolMsg, imageTopics } = this.props.ros
    const status_msg = this.state.status_msg
    const app_namespace = this.getAppNamespace()
    const save_data_topic = this.getSaveNamespace()
    const image_connected = (status_msg !== null) ? (status_msg.image_connected === true) : false

    if (image_connected === false ){
      return (

        <React.Fragment>

        </React.Fragment>

      )
    }
    else {

      // 'None' rather than null: 'None' is the sentinel the viewer and the menu
        // name helpers check for. A null here reached the viewer as a real topic
        // and threw on the first render, before any status had arrived.
        const img_pub_topic = status_msg.image_pub_topic

        // Short <source>-<data_product> form, matching the viewer's own default
        // title and the other apps, instead of the full topic path. Safe on the
        // 'None' fallback above: the helper returns short strings unchanged.
        const img_pub_text = createMenuFirstLastName(img_pub_topic)

        // status_msg is null until the first status arrives; the toggles render
        // unchecked rather than throwing on a null dereference.
        const full_screen_enabled = status_msg.show_full_screen
        const show_targets_enabled = status_msg.show_targets_enabled
        const show_track_enabled = status_msg.show_track_enabled
        const show_goal_enabled = status_msg.show_goal_enabled


        const pantilt_namespace = status_msg.selected_pantilt_topic
        const pantilt_status_msg = status_msg.pantilt_status_msg
        const pantilt_connected = status_msg.pantilt_connected
        const has_abs_pos = pantilt_status_msg.has_absolute_positioning === true
      

        const pan_control_disabled = status_msg.pan_control_disabled === true || pantilt_connected === false
        const pan_control_manaul_enabled = status_msg.pan_control_manaul_enabled
        const pan_control_auto_enabled = status_msg.pan_control_auto_enabled
        const pan_control_namespace = (pan_control_manaul_enabled === true) ? pantilt_namespace : app_namespace
        var pan_goal_ratio = 0.5
        if (pantilt_connected === true) {
            pan_goal_ratio = (pan_control_manaul_enabled === true) ? pantilt_status_msg.pan_goal_ratio : status_msg.auto_pan_ratio 
        }
        const pan_slider_topic = (pan_control_manaul_enabled === true) ? pantilt_namespace + '/goto_pan_ratio'  : app_namespace + '/set_pan_pos_ratio'

        const tilt_control_disabled = status_msg.tilt_control_disabled === true || pantilt_connected === false
        const tilt_control_manaul_enabled = status_msg.tilt_control_manaul_enabled
        const tilt_control_auto_enabled = status_msg.tilt_control_auto_enabled
        const tilt_control_namespace = (tilt_control_manaul_enabled === true) ? pantilt_namespace : app_namespace
        var tilt_goal_ratio = 0.5
        if (pantilt_connected === true) {
            tilt_goal_ratio = (tilt_control_manaul_enabled === true) ? pantilt_status_msg.tilt_goal_ratio : status_msg.auto_tilt_ratio 
        }
        const tilt_slider_topic = (tilt_control_manaul_enabled === true) ? pantilt_namespace + '/goto_tilt_ratio'  : app_namespace + '/set_tilt_pos_ratio'

        // Match the tilt slider to the rendered viewer height. offsetHeight is read
        // off the previous paint, so the first render has no element yet and comes
        // back 1; the sliders stay hidden until a real height exists, which the next
        // status update (1 Hz) triggers.
        const viewerElement = document.getElementById("autoTurretImageViewer")
        const tiltSliderHeight = (viewerElement) ? Math.floor(viewerElement.offsetHeight * 1.0) : 1
        const show_pt_sliders = (tiltSliderHeight === 1) ? false : (pantilt_connected === true && has_abs_pos === true)

          return (
                <Section>


                      <div style={{ display: 'flex' }}>

      

                            <div style={{ width: '10%' }} centered={"true"}>

                                <Label title="Full Screen">
                                  <AsyncToggle
                                    checked={full_screen_enabled === true}
                                    onClick={() => sendBoolMsg(app_namespace + "/set_full_screen", full_screen_enabled === false)}>
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
                                    onClick={() => sendBoolMsg(app_namespace + "/set_show_targets", show_targets_enabled === false)}>
                                  </AsyncToggle>
                                </Label>


                            </div>

                            <div style={{ width: '5%' }} centered={"true"} >
                              {null}
                            </div>

                            <div style={{ width: '10%' }} centered={"true"}>

                              <Label title="Show Track">
                                <AsyncToggle
                                  checked={show_track_enabled === true}
                                  onClick={() => sendBoolMsg(app_namespace + "/set_show_track", show_track_enabled === false)}>
                                </AsyncToggle>
                              </Label>

                            </div>


                        <div style={{ width: '5%' }} centered={"true"} >
                              {null}
                            </div>


                            <div style={{ width: '10%' }} centered={"true"}>

                  <Label title="Show Goal">
                    <AsyncToggle
                      checked={show_goal_enabled === true}
                      onClick={() => sendBoolMsg(app_namespace + "/set_show_crosshair", show_goal_enabled === false)}>
                    </AsyncToggle>
                  </Label>


                            </div>


                            <div style={{ width: '5%' }} centered={"true"} >
                              {null}
                            </div>


                            {/* Image Stab toggle: no image_stab field in AutoTurretStatus yet.
                                A second Full Screen toggle and a source-selector slot were also
                                carried over from the source app; both are dropped -- the Full
                                Screen toggle above is the live one. */}


                      </div>


              <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }} />


              <Columns>
              <Column equalWidth={false}>

                <div id="autoTurretImageViewer">
                  <NepiIFImageViewer
                    image_topic={img_pub_topic}
                    title={''}
                    show_save_controls={false}
                    show_info_controls={false}
                    show_config_controls={false}
                    show_navpose_controls={false}
                    show_render_controls={false}
                    make_section={false}
                    save_data_topic={save_data_topic}
                  />
                </div>

                <div hidden={show_pt_sliders === false}>

                  <SliderAdjustment
                    title={"Pan"}
                    msgType={"std_msgs/Float32"}
                    adjustment={pan_goal_ratio}
                    disabled={pan_control_disabled === true}
                    topic={pan_slider_topic}
                    scaled={0.01}
                    min={0}
                    max={100}
                    tooltip={"Pan as a percentage (0%=min, 100%=max)"}
                    unit={"%"}
                    noTextBox={true}
                    noLabel={true}
                  />

                </div>

              </Column>
              <Column style={{ flex: 0.05 }}>

                <div hidden={show_pt_sliders === false}>

                  <SliderAdjustment
                    title={"Tilt"}
                    msgType={"std_msgs/Float32"}
                    adjustment={tilt_goal_ratio}
                    disabled={tilt_control_disabled === true}
                    topic={tilt_slider_topic}
                    scaled={0.01}
                    min={0}
                    max={100}
                    tooltip={"Tilt as a percentage (0%=min, 100%=max)"}
                    unit={"%"}
                    vertical={true}
                    verticalHeight={tiltSliderHeight}
                    noTextBox={true}
                    noLabel={true}
                  />

                </div>

              </Column>
              </Columns>


                  </Section>
          )
        }
  }






  rendeAutoControls() {
    const { sendBoolMsg } = this.props.ros
    const app_namespace = this.getAppNamespace()

    const status_msg = this.state.status_msg


    if (status_msg == null || app_namespace == null){
      return(

        <Columns>
        <Column>

        </Column>
        </Columns>

      )

    }
    else {
 
        const max_process_rate_hz = status_msg.max_process_rate_hz
        const max_image_pub_rate_hz = status_msg.max_image_pub_rate_hz
        const auto_select_active = status_msg.auto_select_enabled
        const show_control = this.state.show_control
        return (
          <React.Fragment>
   
       
          <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

    
          <Label title={"Process Settings"}></Label>

   



                  <div style={{ display: 'flex' }} >
                    <div style={{ display: "inline-block", width: "20%"}}>{"Scan"}</div>
                    <div style={{ display: "inline-block", width: "5%"}}>{}</div>
                    <div style={{ display: "inline-block", width: "20%"}}>{"Track"}</div>
                    <div style={{ display: "inline-block", width: "5%"}}>{}</div>
                    <div style={{ display: "inline-block", width: "20%" }}>{"Stab"}</div>
                    <div style={{ display: "inline-block", width: "5%"}}>{}</div>
                    <div style={{ display: "inline-block", width: "20%" }}>{"Auto"}</div>
                  </div>

                  <div style={{ display: 'flex' }} >

                  <div style={{ display: "inline-block", width: "20%", float: "left" }}>

                        <Toggle
                        checked={(show_control === 'scan')}
                        onClick={() => onChangeChangeStateValue.bind(this)("show_control",(show_control === 'scan') ? 'None' : 'scan' )}>
                        </Toggle>
                  </div>

                  <div style={{ display: "inline-block", width: "5%"}}>{}</div>


                  <div style={{ display: "inline-block", width: "20%", float: "left" }}>

                        <Toggle
                        checked={(show_control === 'track')}
                        onClick={() => onChangeChangeStateValue.bind(this)("show_control",(show_control === 'track') ? 'None' : 'track' )}>
                        </Toggle>
                  </div>

                  <div style={{ display: "inline-block", width: "5%"}}>{}</div>


                  <div style={{ display: "inline-block", width: "20%", float: "left" }}>

                        <Toggle
                        checked={(show_control === 'stab')}
                        onClick={() => onChangeChangeStateValue.bind(this)("show_control",(show_control === 'stab') ? 'None' : 'stab' )}>
                        </Toggle>
                  </div>

                  <div style={{ display: "inline-block", width: "5%"}}>{}</div>


                    <div style={{ display: "inline-block", width: "20%", float: "left" }}>
                        <Toggle
                          checked={(show_control === 'auto')}
                          onClick={() => onChangeChangeStateValue.bind(this)("show_control",(show_control === 'auto') ? 'None' : 'auto' )}>
                        </Toggle>


                    </div>


              </div>

      { ( show_control !== 'None' ) ?
      <div style={{ borderTop: "1px solid #777777", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }} />

        : null}


      { ( show_control !== 'None' ) ?
           <Label title={show_control.toUpperCase() + ' Process'}></Label>
        : null}
 

      { ( show_control === 'scan' ) ?
      <Nepi_IF_ConnectProcess
        make_section={false}
        title={null}
        namespace={ status_msg.scan_process_namespace}
        allways_show_controls={true}
        />
        : null}


      { ( show_control === 'track' ) ?
      <Nepi_IF_ConnectProcess
        make_section={false}
        title={null}
        namespace={ status_msg.track_process_namespace}
        allways_show_controls={true}
        />
        : null}

      { ( show_control === 'stab' ) ?
      <Nepi_IF_ConnectProcess
        make_section={false}
        title={null}
        namespace={ status_msg.stab_process_namespace}
        allways_show_controls={true}
        />
        : null}





    { ( show_control === 'auto' ) ?
     <SliderAdjustment
          title={"Max Process Rate"}
          msgType={"std_msgs/Float32"}
          adjustment={max_process_rate_hz}
          topic={app_namespace + "/set_max_process_rate"}
          scaled={1.0}
          min={1}
          max={20}
          disabled={false}
          tooltip={"Sets process max rate in hz"}
          unit={"Hz"}
        />
        : null}
{/*         
    { ( show_control === 'auto' ) ?
        <SliderAdjustment
          title={"Max Image Publish Rate"}
          msgType={"std_msgs/Float32"}
          adjustment={max_image_pub_rate_hz}
          topic={app_namespace + "/set_max_image_pub_rate"}
          scaled={1.0}
          min={1}
          max={20}
          disabled={false}
          tooltip={"Sets overlay image max publish rate in hz"}
          unit={"Hz"}
        />
        : null} */}

      { ( show_control === 'auto' ) ?

      <Nepi_IF_ConnectProcess
        make_section={false}
        title={null}
        namespace={ status_msg.auto_process_namespace}
        allways_show_controls={true}
        />
        : null}



          <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }} />

        <Label title={"Process Connections"}></Label>



    

          <Columns>
          <Column>

          </Column>
          <Column>
              <Label title="Auto Select Enable">
                <AsyncToggle
                  checked={auto_select_active === true}
                  onClick={() => sendBoolMsg(app_namespace + "/set_auto_select_enable", !auto_select_active)}>
                </AsyncToggle>
              </Label>

          </Column>
          </Columns>


        <NepiIFConnectPTX
          namespace={app_namespace + "/ptx_connect"}
          select_title={"Pan Tilt"}
          show_selector={true}
          show_controls={false}
          show_settings={false}
          show_data={false}
          make_section={false}
        />

        <NepiIFConnectData
          namespace={app_namespace + "/image_connect"}
          select_title={"Image"}
          show_selector={true}
          show_controls={false}
          show_data={false}
          make_section={false}
        />

        <NepiIFConnectTargets
          namespace={app_namespace + "/targets_connect"}
          select_title={"Targets"}
          show_selector={true}
          show_controls={false}
          show_data={false}
          make_section={false}
        />

        {/* Fourth source row, same shape as the three above. No select_title
            here: Nepi_IF_ConnectNavPose hardcodes its selector label to
            "NavPose Source" and does not read select_title, so passing one
            would be dead weight. Its default (non-shortened, no connect header)
            layout is the same selector-plus-Connected-indicator pair the other
            three rows render, which is why this row carries no BooleanIndicator
            of its own. */}
        <NepiIFConnectNavPose
          namespace={app_namespace + "/navpose_connect"}
          select_title={"NavPose"}
          show_selector={true}
          show_controls={false}
          show_data={false}
          make_section={false}
        />

            </React.Fragment>
        )
  }
}






  // Image viewer and save-data on the left, controls on the right, matching the
  // 75/2/23 split the other apps use. Full screen collapses to the viewer alone.
  render() {
    const status_msg = this.state.status_msg
    const save_data_topic = this.getSaveNamespace()
    const full_screen_enabled = (status_msg !== null) ? status_msg.show_full_screen : false

    if (full_screen_enabled === true) {
      return (
        <React.Fragment>
          {this.renderImageViewer()}
        </React.Fragment>
      )
    }

    return (
      <React.Fragment>

        <div style={{ display: 'flex' }}>

          <div style={{ width: '65%' }}>

            {this.renderImageViewer()}

            {(save_data_topic !== 'None' && this.state.connected === true) ?
              <NepiIFSaveData
                saveNamespace={save_data_topic}
                title={"Nepi_IF_SaveData"}
              />
            : null}

          </div>

          <div style={{ width: '2%' }} centered={"true"} >
            {}
          </div>

          <div style={{ width: '33%' }}>

            {this.renderApp()}

          </div>

        </div>

      </React.Fragment>
    )
  }

}

export default NepiAppAutoTurret
