/*
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi rui (nepi_apps) repo
# (see https://github.com/nepi-engine/nepi_apps)
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
import { Columns, Column } from "./Columns"
import Select, { Option } from "./Select"
import { SliderAdjustment } from "./AdjustmentWidgets"
import BooleanIndicator from "./BooleanIndicator"
import Label from "./Label"
import Input from "./Input"
import Styles from "./Styles"
import Button, { ButtonMenu } from "./Button"
import RangeAdjustment from "./RangeAdjustment"



import {setElementStyleModified, clearElementStyleModified, onChangeSwitchStateValue, onChangeChangeStateValue, onUpdateSetStateValue, round} from "./Utilities"
import {createMenuBaseNames, createMenuFirstLastNames, createMenuListFromStrLists, removeStringFromMenuNames} from "./Utilities"



@inject("ros")
@observer

// Component that contains the SVX controls
class NepiAppSVAutoControls extends Component {
  constructor(props) {
    super(props)

    this.state = {

      appName: 'app_servo_auto',
      appNamespace: null,  
      status_msg: null,  
   
      show_control: 'None',

      selected_servo: 'None',
      panGoto: null,
      panDisabled: null,
      tiltGoto: 0,
      tiltDisabled: null,
      linkSpeeds: true,

      scanPanMin: -50,
      scanPanMax: 50,
      scanTiltMin: -50,
      scanTiltMax: 50,


      track_source_connected: false,
      track_reset_time: null,
      track_goal_deg: null,
      track_move_deg: null,
      tracking_topic: 'track',
      trackPanMin: -50,
      trackPanMax: 50,
      trackTiltMin: -50,
      trackTiltMax: 50,
      trackResetTime: null,

      stab_show_settings: false,
      stab_update_rate: null,
      stab_num_avg: null,
      stab_control_names: [],
      stab_control_values: [],
      stab_reset_time_sec: null,
      stab_pan_pos_max: 0,
      stab_pan_vel_max: 0,
      stab_tilt_pos_max: 0,
      stab_tilt_vel_max: 0,

      lockPanMin: -50,
      lockPanMax: 50,
      lockTiltMin: -50,
      lockTiltMax: 50,



      autoPanEnabled: false,
      autoTiltEnabled: false,

      stabPanReady: false,
      stabTiltReady: false,

      stabPanEnabled: false,
      stabTiltEnabled: false,

      stabPanRunning: false,
      stabTiltRunning: false,

      click_pan_enabled: false,
      click_tilt_enabled: false,

      hide_click_controls: true,

      /*
      sinPanEnabled: false,
      #sinTiltEnabled: false,
      */

      speed_pan_dps: 0,
      speed_tilt_dps: 0,


      statusListener: null,         
      needs_update: false

    }


    this.onEnterSendPanScanRangeWindowValue = this.onEnterSendPanScanRangeWindowValue.bind(this)
    this.onEnterSendTiltScanRangeWindowValue = this.onEnterSendTiltScanRangeWindowValue.bind(this)

    this.renderSVAuto = this.renderSVAuto.bind(this)
    this.renderSVControls = this.renderSVControls.bind(this)
    this.renderScanControls = this.renderScanControls.bind(this)


    this.onUpdateInputControlNameValue = this.onUpdateInputControlNameValue.bind(this)
    this.onKeySaveInputControlNameValue = this.onKeySaveInputControlNameValue.bind(this)
    this.renderStabControlValues = this.renderStabControlValues.bind(this)
    this.renderStabControls = this.renderStabControls.bind(this)
    this.onStabUpdateText = this.onStabUpdateText.bind(this)
    this.onStabKeyText = this.onStabKeyText.bind(this)

    this.onMenuSelection = this.onMenuSelection.bind(this)
    this.renderTrackControls = this.renderTrackControls.bind(this)

    this.onSVUpdateText = this.onSVUpdateText.bind(this)
    this.onSVKeyText = this.onSVKeyText.bind(this)
    this.renderSVAutoControls = this.renderSVAutoControls.bind(this)

    this.onTrackUpdateText = this.onTrackUpdateText.bind(this)
    this.onTrackKeyText = this.onTrackKeyText.bind(this)


    this.getNamespace = this.getNamespace.bind(this)
    this.getBaseNamespace = this.getBaseNamespace.bind(this)
    this.updateStatusListener = this.updateStatusListener.bind(this)
    this.statusListener = this.statusListener.bind(this)

  }

  getNamespace(){
    const { namespacePrefix, deviceId} = this.props.ros
    var namespace = null
    if (namespacePrefix != null && deviceId != null){
      if (this.props.namespace !== undefined){
        namespace = this.props.namespace
      }
    }
    return namespace
  }

  getBaseNamespace(){
    const { namespacePrefix, deviceId} = this.props.ros
    var baseNamespace = null
    if (namespacePrefix !== null && deviceId !== null){
      baseNamespace = "/" + namespacePrefix + "/" + deviceId 
    }
    return baseNamespace
  }


  // Callback for handling ROS Status3DX messages
  statusListener(message) {
    const last_status_msg = this.state.status_msg
    this.setState({
      status_msg: message,
      selected_servo: message.selected_servo,

      autoPanEnabled: message.scan_pan_enabled,
      autoTiltEnabled: message.scan_tilt_enabled,

      track_source_connected: message.track_source_connected,

      trackPanEnabled: message.track_pan_enabled,
      trackTiltEnabled: message.track_tilt_enabled,
      
      stabPanReady: message.stab_pan_ready,
      stabTiltReady: message.stab_tilt_ready,

      stabPanEnabled: message.stab_pan_enabled,
      stabTiltEnabled: message.stab_tilt_enabled,

      stabPanRunning: message.pan_stabing,
      stabTiltRunning: message.tilt_stabing,

      sinPanEnabled: message.sin_pan_enabled,
      sinTiltEnabled: message.sin_tilt_enabled,

      click_pan_enabled: message.click_pan_enabled,
      click_tilt_enabled: message.click_tilt_enabled,

      
    })
  
    const pan_min_scan = round(message.scan_pan_min_deg, 0)
    const pan_max_scan = round(message.scan_pan_max_deg, 0)
    const tilt_min_scan = round(message.scan_tilt_min_deg, 0)
    const tilt_max_scan = round(message.scan_tilt_max_deg, 0)

    var scan_limits_changed = true
    if (last_status_msg != null) {


      const last_pan_min_scan = round(message.scan_pan_min_deg, 0)
      const last_pan_max_scan = round(message.scan_pan_max_deg, 0)
      const last_tilt_min_scan = round(message.scan_tilt_min_deg, 0)
      const last_tilt_max_scan = round(message.scan_tilt_max_deg, 0)

      scan_limits_changed = (pan_min_scan !== last_pan_min_scan || pan_max_scan !== last_pan_max_scan ||
                                tilt_min_scan !== last_tilt_min_scan || tilt_max_scan !== last_tilt_max_scan)
      }
      
    if (scan_limits_changed === true){
      this.setState({scanPanMin: pan_min_scan,
                    scanPanMax: pan_max_scan
      })
    }
    if (scan_limits_changed === true){
      this.setState({scanTiltMin: tilt_min_scan,
                    scanTiltMax: tilt_max_scan
      })
    }

    var stab_controls_changed = true
   if (last_status_msg != null) {
      stab_controls_changed = JSON.stringify(message.stab_control_values) !== JSON.stringify(last_status_msg.stab_control_values)
   }
    
    if (stab_controls_changed === true){
      this.setState({stab_control_names: message.stab_control_names,
                    stab_control_values: message.stab_control_values
      })
    }


  }




  // Function for configuring and subscribing to Status
  updateStatusListener(namespace) {
    
    if (this.state.statusListener != null) {
      this.state.statusListener.unsubscribe()
      this.setState({statusListener: null, status_msg: null})
    }
    if (namespace != null && namespace !== 'None' && namespace.indexOf('null') === -1){
        const statusNamespace = namespace + '/status'
        var statusListener = this.props.ros.setupStatusListener(
              statusNamespace,
              "nepi_app_servo_auto/ServoAutoAppStatus",
              this.statusListener
            )
    this.setState({ 
      statusListener: statusListener,
    })
    }
    this.setState({ 
      appNamespace: namespace,
      needs_update: false
    })
  }
  
// Lifecycle method called when compnent updates.
// Used to track changes in the topic
componentDidUpdate(prevProps, prevState, snapshot) {
  const namespace = this.getNamespace()
  if ((namespace != null && namespace !== this.state.appNamespace) || this.state.needs_update === true){
      this.updateStatusListener(namespace)
  }
}

  componentDidMount() {
    this.setState({ needs_update: true })
    }
  // Lifecycle method called just before the component umounts.
  // Used to unsubscribe to Status3DX message


componentWillUnmount() {
    if (this.state.statusListener) {
      this.state.statusListener.unsubscribe()
      this.setState({statusListener : null})
    }
  }






  onSVUpdateText(e) {
    var panElement = null
    var tiltElement = null
    if (e.target.id === "SVXPanGoto") 
      {
        panElement = document.getElementById("SVXPanGoto")
        setElementStyleModified(panElement)
        this.setState({panGoto: e.target.value})
             
      }
        
    else if  (e.target.id === "SVXTiltGoto")
        {
          tiltElement = document.getElementById("SVXTiltGoto")
          setElementStyleModified(tiltElement)
          this.setState({tiltGoto: e.target.value})         
          
        }

  }

  onSVKeyText(e) {
    const {svxDevices, onSetSVXGotoPos, onSetSVXGotoPanPos, onSetSVXGotoTiltPos, onSetSVXHomePos, onSetSVXSoftStopPos} = this.props.ros

    const selected_servo = this.state.selected_servo
    const svx_caps = svxDevices[selected_servo]
    const has_timed_pos = svx_caps && (svx_caps.has_timed_positioning)
    const has_sep_servo = svx_caps && (svx_caps.has_seperate_servo_control)
    const has_abs_pos = svx_caps && (svx_caps.has_absolute_positioning === true)
    const has_homing = svx_caps && (svx_caps.has_homing)
    const has_speed_control = svx_caps && (svx_caps.has_adjustable_speed)
    const has_sep_speed = svx_caps && (svx_caps.has_seperate_servo_speed === true)
    //Unused const has_set_home = svx_caps && (svx_caps.has_set_home)

    var panElement = null
    var tiltElement = null
    if(e.key === 'Enter'){
      if (e.target.id === "SVXPanGoto") 
        {
          panElement = document.getElementById("SVXPanGoto")
          tiltElement = document.getElementById("SVXTiltGoto")                    
          if (has_sep_servo === true){
            onSetSVXGotoPanPos(selected_servo, Number(panElement.value))
          }
          else {
            onSetSVXGotoPos(selected_servo, Number(panElement.value),Number(tiltElement.value))
          }            
          clearElementStyleModified(panElement)   
          this.setState({panGoto: null})    
          
        }
        else if  (e.target.id === "SVXTiltGoto")
          {
            
            panElement = document.getElementById("SVXPanGoto")
            tiltElement = document.getElementById("SVXTiltGoto")

            if (has_sep_servo === true){
              onSetSVXGotoTiltPos(selected_servo, Number(tiltElement.value))
            }
            else {
              onSetSVXGotoPos(selected_servo, Number(panElement.value),Number(tiltElement.value))
            }              
            clearElementStyleModified(tiltElement)
            this.setState({tiltGoto: null})      
          
          }
    }
  }




  renderSVAuto() {
    const { svxDevices, sendBoolMsg } = this.props.ros

    const { autoPanEnabled, autoTiltEnabled, trackPanEnabled, trackTiltEnabled,
            track_source_connected, stabPanEnabled, stabTiltEnabled,
            click_pan_enabled, click_tilt_enabled  } = this.state /*sinPanEnabled ,sinTiltEnabled*/

    const selected_servo = this.state.selected_servo

    //Unused const {sendTriggerMsg} = this.props.ros

    const namespace = this.getNamespace()

    const status_msg = this.state.status_msg
    const topics = Object.keys(svxDevices)
    const sv_connected_topics = []
    var i
    for (i = 0; i <topics.length; i++) {
    if (topics[i].includes(selected_servo)){
      sv_connected_topics.push(topics[i])
    }
  }
    
    const sv_connected = (sv_connected_topics.indexOf(selected_servo) !== -1)
    //console.log('sv_connected: ' + sv_connected)


    if (status_msg == null || sv_connected == false){
      return(

        <Columns>
        <Column>

        </Column>
        </Columns>

      )

    }
    else {


    const has_scan_pan = (status_msg.sv_status_msg.has_scan_pan)
    const has_scan_tilt = (status_msg.sv_status_msg.has_scan_tilt)
    const has_abs_pos = (status_msg.sv_status_msg.has_absolute_positioning)
    const has_homing = (status_msg.sv_status_msg.has_homing)
    const has_speed_control = (status_msg.sv_status_msg.has_adjustable_speed)
    const has_sep_speed = (status_msg.sv_status_msg.has_seperate_servo_speed)

    const disable_track_enable = ((track_source_connected === false || has_scan_pan === false || has_scan_tilt === false))

    const disable_stab_enable = false

      const pan_control_disabled = (status_msg.pan_control_disabled === true)
      const tilt_control_disabled = (status_msg.tilt_control_disabled === true)
      const speedRatio = status_msg.speed_ratio
      const speedPanRatio = status_msg.pan_speed_ratio
      const speedTiltRatio = status_msg.tilt_speed_ratio


      const show_control = this.state.show_control
        return (
          <React.Fragment>



          { (has_homing === false) ?


          <ButtonMenu>
            <Button onClick={() => this.props.ros.onSVXStop(selected_servo)}>{"STOP"}</Button>
          </ButtonMenu>

          :

          <ButtonMenu>
            <Button onClick={() => this.props.ros.onSVXStop(selected_servo)}>{"STOP"}</Button>
            <Button onClick={() => this.props.ros.sendTriggerMsg(namespace + '/pan_home')}>{"PAN HOME"}</Button>
            <Button onClick={() => this.props.ros.sendTriggerMsg(namespace + '/tilt_home')}>{"TILT HOME"}</Button>
          </ButtonMenu>

          }

          <Label title={""} style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
            <div style={{ display: "inline-block", width: "45%", float: "left" }}>{"Pan"}</div>
            <div style={{ display: "inline-block", width: "45%", float: "left" }}>{"Tilt"}</div>
          </Label>


            <Label title={"Enable Scanning"}>
              <div style={{ display: "inline-block", width: "45%", float: "left" }}>
                <Toggle style={{justifyContent: "flex-left"}} 
                  checked={autoPanEnabled} 
                  onClick={() => sendBoolMsg.bind(this)(namespace + "/set_scan_pan_enable",!autoPanEnabled)} />
              </div>


              <div style={{ display: "inline-block", width: "45%", float: "right" }}>
                <Toggle style={{justifyContent: "flex-right"}} 
                  checked={autoTiltEnabled} 
                  onClick={() => sendBoolMsg.bind(this)(namespace + "/set_scan_tilt_enable",!autoTiltEnabled)} />
              </div>

            </Label>


              <Label title={"Enable Tracking"}>

                <div style={{ display: "inline-block", width: "45%", float: "left" }}>
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
                </div>

              </Label>


              <Label title={"Enable Stabilize"}>

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
                </div>

              </Label>



            {this.renderSVControls()}

            </React.Fragment>
        )
  }
}




  onClickToggleShowSettings(){
    const currentVal = this.state.showSettings 
    this.setState({showSettings: !currentVal})
    this.render()
  }




  renderSVAutoControls() {
    const { svxDevices, sendBoolMsg } = this.props.ros

    const { autoPanEnabled, autoTiltEnabled, trackPanEnabled, trackTiltEnabled,
            track_source_connected, stabPanEnabled, stabTiltEnabled,
            click_pan_enabled, click_tilt_enabled  } = this.state /*sinPanEnabled ,sinTiltEnabled*/

    const selected_servo = this.state.selected_servo

    //Unused const {sendTriggerMsg} = this.props.ros

    const namespace = this.getNamespace()

    const status_msg = this.state.status_msg


    if (status_msg == null || namespace == null){
      return(

        <Columns>
        <Column>

        </Column>
        </Columns>

      )

    }
    else {
 


        const show_control = this.state.show_control
        return (
          <React.Fragment>
   

          { this.renderSVAuto() }
        

          <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>





        <Label title={"Show Controls"}></Label>

        <div style={{ display: 'flex' }} >
           <div style={{ display: "inline-block", width: "20%"}}>{""}</div>
           <div style={{ display: "inline-block", width: "5%"}}>{}</div>
          <div style={{ display: "inline-block", width: "20%"}}>{"Scan"}</div>
          <div style={{ display: "inline-block", width: "5%"}}>{}</div>
          <div style={{ display: "inline-block", width: "20%" }}>{"Track"}</div>
          <div style={{ display: "inline-block", width: "5%"}}>{}</div>
          <div style={{ display: "inline-block", width: "20%" }}>{"Stab"}</div>
        </div>

        <div style={{ display: 'flex' }} >

          <div style={{ display: "inline-block", width: "20%", float: "left" }}>
              {/* <Toggle
              checked={(show_control === 'sv')}
              onClick={() => onChangeChangeStateValue.bind(this)("show_control",(show_control === 'sv') ? 'None' : 'sv' )}>
              </Toggle> */}
          </div>

          <div style={{ display: "inline-block", width: "5%"}}>{}</div>

          <div style={{ display: "inline-block", width: "20%", float: "left" }}>
              <Toggle
              checked={(show_control === 'scan')}
              onClick={() => onChangeChangeStateValue.bind(this)("show_control",(show_control === 'scan') ? 'None' : 'scan' )}>
              </Toggle>
          </div>

          <div style={{ display: "inline-block", width: "5%"}}>{}</div>

          <div style={{ display: "inline-block", width: "20%", float: "center" }}>
              <Toggle
                checked={(show_control === 'track')}
                onClick={() => onChangeChangeStateValue.bind(this)("show_control",(show_control === 'track') ? 'None' : 'track' )}>
              </Toggle>
          </div>

          <div style={{ display: "inline-block", width: "5%"}}>{}</div>

          <div style={{ display: "inline-block", width: "20%", float: "right" }}>
              <Toggle
                checked={(show_control === 'stab')}
                onClick={() => onChangeChangeStateValue.bind(this)("show_control",(show_control === 'stab') ? 'None' : 'stab' )}>
              </Toggle>
          </div>

        </div>
         
          {/* <div hidden={(show_control !== 'sv')}>
                {this.renderSVControls()}
          </div> */}


          <div hidden={(show_control !== 'scan')}>
                {this.renderScanControls()}
          </div>

         
          <div hidden={(show_control !== 'track')}>
                {this.renderTrackControls()}
          </div>

       
          <div hidden={(show_control !== 'stab')}>
                {this.renderStabControls()}
          </div>

            </React.Fragment>
        )
  }
}





  renderSVControls() {
    const { svxDevices, sendBoolMsg } = this.props.ros

    const { autoPanEnabled, autoTiltEnabled, trackPanEnabled, trackTiltEnabled,
            track_source_connected, stabPanEnabled, stabTiltEnabled,
            click_pan_enabled, click_tilt_enabled  } = this.state /*sinPanEnabled ,sinTiltEnabled*/

    const selected_servo = this.state.selected_servo

    //Unused const {sendTriggerMsg} = this.props.ros

    const namespace = this.getNamespace()

    const status_msg = this.state.status_msg
    const topics = Object.keys(svxDevices)
    const sv_connected_topics = []
    var i
    for (i = 0; i <topics.length; i++) {
    if (topics[i].includes(selected_servo)){
      sv_connected_topics.push(topics[i])
    }
  }
    
    const sv_connected = (sv_connected_topics.indexOf(selected_servo) !== -1)
    //console.log('sv_connected: ' + sv_connected)


    if (status_msg == null || sv_connected == false){
      return(

        <Columns>
        <Column>

        </Column>
        </Columns>

      )

    }
    else {


    const has_scan_pan = (status_msg.sv_status_msg.has_scan_pan)
    const has_scan_tilt = (status_msg.sv_status_msg.has_scan_tilt)
    const has_abs_pos = (status_msg.sv_status_msg.has_absolute_positioning)
    const has_homing = (status_msg.sv_status_msg.has_homing)
    const has_speed_control = (status_msg.sv_status_msg.has_adjustable_speed)
    const has_sep_speed = (status_msg.sv_status_msg.has_seperate_servo_speed)

    const disable_track_enable = ((track_source_connected === false || has_scan_pan === false || has_scan_tilt === false))

    const disable_stab_enable = false

      const panPosition = status_msg.sv_status_msg.pan_now_deg
      const tiltPosition = status_msg.sv_status_msg.tilt_now_deg

      const panPositionClean = panPosition + .001
      const tiltPositionClean = tiltPosition + .001

      if (this.state.panGoto == null){
        this.setState({panGoto: panPositionClean})
      }

      if (this.state.tiltGoto == null){
        this.setState({tiltGoto: tiltPositionClean})
      }

      const panMove = status_msg.sv_status_msg.pan_goal_deg
      const tiltMove = status_msg.sv_status_msg.tilt_goal_deg

      const panMoveClean = panMove + .001
      const tiltMoveClean = tiltMove + .001

      const pan_control_disabled = (status_msg.pan_control_disabled === true)
      
      if (pan_control_disabled !== this.state.panDisabled){
        this.setState({panGoto: panMoveClean, panDisabled: pan_control_disabled})
      }
      const pan_pos = pan_control_disabled === true ? panMoveClean : this.state.panGoto

      const tilt_control_disabled = (status_msg.tilt_control_disabled === true)
      if (tilt_control_disabled !== this.state.tiltDisabled){
        this.setState({tiltGoto: tiltMoveClean, tiltDisabled: tilt_control_disabled})
      }      
      const tilt_pos = tilt_control_disabled === true ? tiltMoveClean : this.state.tiltGoto


      const panCurSpeed = status_msg.sv_status_msg.speed_pan_dps
      const tiltCurSpeed = status_msg.sv_status_msg.speed_tilt_dps

      const panCurSpeedClean = panCurSpeed + .001
      const tiltCurSpeedClean = tiltCurSpeed + .001

      const speedRatio = status_msg.speed_ratio
      const speedPanRatio = status_msg.pan_speed_ratio
      const speedTiltRatio = status_msg.tilt_speed_ratio

      const maxSpeed = status_msg.servo_max_speed_dps
      const panSetSpeed = speedPanRatio * maxSpeed
      const tiltSetSpeed = speedTiltRatio * maxSpeed

      const panSetSpeedClean = panSetSpeed + .001
      const tiltSetSpeedClean = tiltSetSpeed + .001

        return (
          <React.Fragment>


          <div hidden={(has_abs_pos === false)}>


              <Label title={"GoTo Position "}>
                <Input
                  disabled={pan_control_disabled === true}
                  id={"SVXPanGoto"}
                  style={{ width: "45%", float: "left" }}
                  value={round(pan_pos,1)}
                  onChange= {this.onSVUpdateText}
                  onKeyDown= {this.onSVKeyText}
                />
                <Input
                  disabled={tilt_control_disabled === true}
                  id={"SVXTiltGoto"}
                  style={{ width: "45%" }}
                  value={round(tilt_pos,1)}
                  onChange= {this.onSVUpdateText}
                  onKeyDown= {this.onSVKeyText}
                />
              </Label>


              <Label title={"Current Position"}>
                <Input
                  disabled
                  style={{ width: "45%", float: "left" }}
                  value={round(panPositionClean, 1)}
                />
                <Input
                  disabled
                  style={{ width: "45%" }}
                  value={round(tiltPositionClean, 1)}
                />
              </Label>

              <Label title={"Average Speed"}>
                <Input
                  disabled
                  style={{ width: "45%", float: "left" }}
                  value={round(panCurSpeedClean, 1)}
                />
                <Input
                  disabled
                  style={{ width: "45%" }}
                  value={round(tiltCurSpeedClean, 1)}
                />
              </Label>

  
          </div>



          <div hidden={(has_speed_control === false)}>

              <React.Fragment>
                <SliderAdjustment
                  disabled={pan_control_disabled}
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
                  disabled={tilt_control_disabled}
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



onEnterSendPanScanRangeWindowValue(event, topicName, entryName, other_val) {
  const {publishRangeWindow} = this.props.ros
  const appnamespace = this.getNamespace()
  const topic_namespace = appnamespace + '/' + topicName
  var min = 0
  var max = 0
  if(event.key === 'Enter'){
    const value = parseFloat(event.target.value)
    if (entryName === "max"){
      if (value < this.state.scanPanMin && !isNaN(value)){
        console.log("invaled range")

        const cur_max = this.state.status_msg.scan_pan_max_deg
        this.setState({scanPanMax: cur_max })
      }
      else{
        min = other_val
        max = value
        publishRangeWindow(topic_namespace,min,max,false)
      }
    }
    else if (entryName === "min") {
      if (value > this.state.scanPanMax && !isNaN(value)){
        console.log("invaled range")

        const cur_min = this.state.status_msg.scan_pan_min_deg
        this.setState({scanPanMin: cur_min })
      }
      else {
        min = value
        max = other_val
        publishRangeWindow(topic_namespace,min,max,false)
    }
    }
  
    document.getElementById(event.target.id).style.color = Styles.vars.colors.black
  }

}

onEnterSendTiltScanRangeWindowValue(event, topicName, entryName, other_val) {
  const {publishRangeWindow} = this.props.ros
  const appnamespace = this.getNamespace()
  const topic_namespace = appnamespace + '/' + topicName
  var min = 0
  var max = 0
  if(event.key === 'Enter'){
    const value = parseFloat(event.target.value)
    if (entryName === "max"){
      if (value < this.state.scanTiltMin && !isNaN(value)){
        console.log("invaled range")

        const cur_max = this.state.status_msg.scan_tilt_max_deg
        this.setState({scanTiltMax: cur_max })
      }
      else{
        min = other_val
        max = value
        publishRangeWindow(topic_namespace,min,max,false)
      }
    }
    else if (entryName === "min") {
      if (value > this.state.scanTiltMax && !isNaN(value)){
        console.log("invaled range")

        const cur_min = this.state.status_msg.scan_tilt_min_deg
        this.setState({scanTiltMin: cur_min })
      }
      else {
        min = value
        max = other_val
        publishRangeWindow(topic_namespace,min,max,false)
    }
    }
  
    document.getElementById(event.target.id).style.color = Styles.vars.colors.black
  }

}

  renderScanControls() {


    const namespace = this.getNamespace()

    const status_msg = this.state.status_msg

   

        return (
          <React.Fragment>


 <div style={{ borderTop: "1px solid #777777", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>
{/* 
          <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
                {"Servo Frame - Angles in ENU frame (Tilt+:Down , Pan+:Left)"}
              </label> */}



              <React.Fragment>
                <SliderAdjustment
                  disabled={false}
                  title={"Pan Speed"}
                  msgType={"std_msgs/Float32"}
                  adjustment={status_msg.scan_pan_speed_ratio}
                  topic={namespace + "/set_scan_pan_speed_ratio"}
                  scaled={0.01}
                  min={0}
                  max={100}
                  tooltip={"Speed as a percentage (0%=min, 100%=max)"}
                  unit={"%"}
                />
                <SliderAdjustment
                  disabled={false}
                  title={"Tilt Speed"}
                  msgType={"std_msgs/Float32"}
                  adjustment={status_msg.scan_tilt_speed_ratio}
                  topic={namespace + "/set_scan_tilt_speed_ratio"}
                  scaled={0.01}
                  min={0}
                  max={100}
                  tooltip={"Speed as a percentage (0%=min, 100%=max)"}
                  unit={"%"}
                />
              </React.Fragment>
  



            <Label title={""}>
              <div style={{ display: "inline-block", width: "45%", float: "left" }}>{"Pan"}</div>
              <div style={{ display: "inline-block", width: "45%", float: "left" }}>{"Tilt"}</div>
            </Label>

            <Label title={"Min Scan Limit"}>

              <Input id="scan_pan_min" 
                  value={this.state.scanPanMin} 
                  style={{ width: "45%", float: "left" }}
                  onChange={(event) => onUpdateSetStateValue.bind(this)(event,"scanPanMin")} 
                  onKeyDown= {(event) => this.onEnterSendPanScanRangeWindowValue(event,"/set_scan_pan_window","min",Number(this.state.scanPanMax))} />

              <Input id="scan_tilt_min" 
                  value={this.state.scanTiltMin} 
                  style={{ width: "45%" }}
                  onChange={(event) => onUpdateSetStateValue.bind(this)(event,"scanTiltMin")} 
                  onKeyDown= {(event) => this.onEnterSendTiltScanRangeWindowValue(event,"/set_scan_tilt_window","min",Number(this.state.scanTiltMax))} />

              
            </Label>


            <Label title={"Max Scan Limit"}>

              <Input id="scan_pan_max" 
                value={this.state.scanPanMax} 
                style={{ width: "45%", float: "left" }}
                onChange={(event) => onUpdateSetStateValue.bind(this)(event,"scanPanMax")} 
                onKeyDown= {(event) => this.onEnterSendPanScanRangeWindowValue(event,"/set_scan_pan_window","max",Number(this.state.scanPanMin))} />     


              <Input id="scan_tilt_max" 
                  value={this.state.scanTiltMax} 
                  style={{ width: "45%" }}
                  onChange={(event) => onUpdateSetStateValue.bind(this)(event,"scanTiltMax")} 
                  onKeyDown= {(event) => this.onEnterSendTiltScanRangeWindowValue(event,"/set_scan_tilt_window","max",Number(this.state.scanTiltMin))} />                      
            </Label>





            </React.Fragment>
        )
  
}


  onMenuSelection(event){
    const {sendStringMsg} = this.props.ros

    const value = event.target.value
    const topic = event.target.id
    const namespace = this.getNamespace()
    const tracking_topic = this.state.tracking_topic
    const sendNamespace = namespace + '/' + tracking_topic + '/' +  topic
    sendStringMsg(sendNamespace,value)
  }



  onTrackUpdateText(e) {
    var element = null
    if (e.target.id === "ErrorGoalDeg")
      {
        element = document.getElementById("ErrorGoalDeg")
        setElementStyleModified(element)
        this.setState({track_goal_deg: e.target.value})
      }
    else if (e.target.id === "MinMoveDeg")
      {
        element = document.getElementById("MinMoveDeg")
        setElementStyleModified(element)
        this.setState({track_move_deg: e.target.value})
      }
    else if (e.target.id === "LostTargetTime")
      {
        element = document.getElementById("LostTargetTime")
        setElementStyleModified(element)
        this.setState({track_reset_time: e.target.value})
      }
  }

  onTrackKeyText(e) {
    const {sendFloatMsg} = this.props.ros
    const namespace = this.getNamespace()

    var element = null

    if(e.key === 'Enter'){
      if (e.target.id === "ErrorGoalDeg")
        {
          element = document.getElementById("ErrorGoalDeg")
          clearElementStyleModified(element)
          const goal_deg = parseFloat(e.target.value)
          if (!isNaN(goal_deg)){
            sendFloatMsg(namespace + "/set_track_goal_deg", goal_deg)
          }
          this.setState({track_goal_deg: null})
        }
      else if (e.target.id === "MinMoveDeg")
        {
          element = document.getElementById("MinMoveDeg")
          clearElementStyleModified(element)
          const move_deg = parseFloat(e.target.value)
          if (!isNaN(move_deg)){
            sendFloatMsg(namespace + "/set_track_move_deg", move_deg)
          }
          this.setState({track_move_deg: null})
        }
      else if (e.target.id === "LostTargetTime")
        {
          element = document.getElementById("LostTargetTime")
          clearElementStyleModified(element)
          this.setState({track_reset_time: null})
          const lost_time = parseFloat(e.target.value)
          if (!isNaN(lost_time)){
            sendFloatMsg(namespace + "/set_track_reset_time_sec", lost_time)
          }
        }
    }
  }




  onStabUpdateText(e) {
    var element = null
    const id = e.target.id
    const stateKey = {
      StabUpdateRate: 'stab_update_rate',
      StabNumAvg: 'stab_num_avg',
      StabPosDeg: 'stab_pos_deg',
      StabVelDeg: 'stab_vel_deg',
      StabResetTimeSec: 'stab_reset_time_sec'
    }[id]
    if (stateKey) {
      element = document.getElementById(id)
      setElementStyleModified(element)
      this.setState({[stateKey]: e.target.value})
    }
  }

  onStabKeyText(e) {
    const {sendFloatMsg, sendIntMsg} = this.props.ros
    const namespace = this.getNamespace()
    var element = null
    if (e.key === 'Enter') {
      const val = parseFloat(e.target.value)
      if (e.target.id === "StabUpdateRate") {
        element = document.getElementById("StabUpdateRate")
        clearElementStyleModified(element)
        if (!isNaN(val)) { sendFloatMsg(namespace + "/set_stab_update_rate", val) }
        this.setState({stab_update_rate: null})
      } else if (e.target.id === "StabNumAvg") {
        element = document.getElementById("StabNumAvg")
        clearElementStyleModified(element)
        if (!isNaN(val)) { sendIntMsg(namespace + "/set_stab_num_avg", Math.round(val)) }
        this.setState({stab_num_avg: null})
      } else if (e.target.id === "StabPosDeg") {
        element = document.getElementById("StabPosDeg")
        clearElementStyleModified(element)
        if (!isNaN(val)) { sendFloatMsg(namespace + "/set_stab_pos_deg", val) }
        this.setState({stab_pos_deg: null})
      } else if (e.target.id === "StabVelDeg") {
        element = document.getElementById("StabVelDeg")
        clearElementStyleModified(element)
        if (!isNaN(val)) { sendFloatMsg(namespace + "/set_stab_vel_deg", val) }
        this.setState({stab_vel_deg: null})
      } else if (e.target.id === "StabResetTimeSec") {
        element = document.getElementById("StabResetTimeSec")
        clearElementStyleModified(element)
        if (!isNaN(val)) { sendFloatMsg(namespace + "/set_stab_reset_time_sec", val) }
        this.setState({stab_reset_time_sec: null})
      }
    }
  }

  renderTrackControls() {


    const namespace = this.getNamespace()
    const track_namespace = namespace + '/' + this.state.tracking_topic

    const status_msg = this.state.status_msg
    const track_status_msg = status_msg.tracking_status

    const enabled = track_status_msg.enabled
    const running = track_status_msg.running
    const state = track_status_msg.state

    const available_targets = track_status_msg.available_targets_topics
    const has_targets = available_targets.length > 0
    const targets_names = createMenuBaseNames(available_targets)
    const targets_menu = createMenuListFromStrLists(available_targets,targets_names, ['None'], [],'None Available')
    const selected_targets = track_status_msg.selected_targets
    const targets_connected = track_status_msg.targets_connected

    
    const available_sources = track_status_msg.available_source_topics
    const sources_names = createMenuFirstLastNames(available_sources)
    const sources_menu = createMenuListFromStrLists(available_sources,sources_names, ['None'], [],'None Available')
    const selected_source = track_status_msg.selected_source
    const source_connected = track_status_msg.source_connected

    const available_classes = track_status_msg.available_classes
    const classes_menu = createMenuListFromStrLists(available_classes,available_classes, ['None'], [],'None Available')
    const selected_class = track_status_msg.selected_class
    const class_selected = (selected_class !== 'None')

    const threshold_filter = track_status_msg.threshold_filter

    const available_best = track_status_msg.available_best_filters
    const best_menu = createMenuListFromStrLists(available_best,available_best, [], [], '')
    const best_filter = track_status_msg.selected_best_filter


    const goal_deg = status_msg.track_goal_deg
    var track_goal_deg = this.state.track_goal_deg
    if (track_goal_deg == null){
      track_goal_deg = goal_deg
    }

    const move_deg = status_msg.track_move_deg
    var track_move_deg = this.state.track_move_deg
    if (track_move_deg == null){
      track_move_deg = move_deg
    }


    const move_ratio = status_msg.track_move_ratio
    const reset_time = status_msg.track_reset_time_sec
    var track_reset_time = this.state.track_reset_time
    if (track_reset_time == null){
      track_reset_time = reset_time
    }
    const track_pan_error = status_msg.track_pan_error
    const track_tilt_error = status_msg.track_tilt_error


    const manages_targeting = track_status_msg.manages_targeting

        return (
          <React.Fragment>

    <div hiddent={has_targets === false}>

 <div style={{ borderTop: "1px solid #777777", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

                 <div style={{ display: 'flex' }}>
                      <div style={{ width: '10%' }}>

                      </div>

                      <div style={{ width: '30%' }}>


                          <Label title={"Targeting"}>
                            <BooleanIndicator value={running} />
                          </Label>


                      </div>

                      <div style={{ width: '10%' }}>

                      </div>

                      <div style={{ width: '30%' }} >

                          <Label title={"Tracking"}>
                            <BooleanIndicator value={state} />
                          </Label>

                      </div>

                      <div style={{ width: '10%' }}>

                      </div>
                </div>               

                <Label title={"Target Angles"}>
                    <Input
                      disabled
                      style={{ width: "45%", float: "left" }}
                      value={round(track_pan_error, 2)}
                    />

                    <Input
                      disabled
                      style={{ width: "45%" }}
                      value={round(track_tilt_error, 2)}
                    />
                </Label>

{/* 

                <div style={{ display: 'flex' }}>
                      <div style={{ width: '30%' }}>

                      <Label title={"Targets"}>
                    <BooleanIndicator value={targets_connected} />
                    </Label>

                      </div>

                      <div style={{ width: '30%' }}>

                       <Label title={"Image"}>
                      <BooleanIndicator value={source_connected  && targets_connected} />
                      </Label>

                      </div>

                      <div style={{ width: '30%' }} >


                      <Label title={"Class"}>
                     <BooleanIndicator value={class_selected && targets_connected } />
                     </Label>


                      </div>
          </div> */}

        <div style={{ borderTop: "1px solid #777777", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

            <Label title={'Select Source'}>
              <Select
                id="set_targets_topic"
                onChange={this.onMenuSelection}
                value={selected_targets}
              >
                {targets_menu}
              </Select>
            </Label>


        

              <Label title={'Select Image'}>
                <Select
                  id="set_source_topic"
                  onChange={this.onMenuSelection}
                  value={selected_source}
                >
                  {sources_menu}
                </Select>
              </Label>



              <Label title={'Select Class'}>
                <Select
                  id="set_class_filter"
                  onChange={this.onMenuSelection}
                  value={selected_class}
                >
                  {classes_menu}
                </Select>
              </Label>



                    <Label title={'Select Filter'}>
                      <Select
                        id="set_best_filter"
                        onChange={this.onMenuSelection}
                        value={best_filter}
                      >
                        {best_menu}
                      </Select>
                    </Label>

               

            <div style={{ borderTop: "1px solid #000000", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

                  <SliderAdjustment
                            title={"Threshold"}
                            msgType={"std_msgs/Float32"}
                            adjustment={threshold_filter}
                            topic={track_namespace + "/set_threshold_filter"}
                            scaled={0.01}
                            min={0}
                            max={100}
                            disabled={false}
                            tooltip={"Sets target confidence threshold filter"}
                            unit={"%"}
                    />


 <div style={{ borderTop: "1px solid #777777", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>



          <Columns>
          <Column>

             <Label title={"Goal Deg"}>
                <Input
                  id={"ErrorGoalDeg"}
                  style={{ width: "45%", float: "left" }}
                  value={track_goal_deg}
                  onChange= {this.onTrackUpdateText}
                  onKeyDown= {this.onTrackKeyText}
                />
              </Label>

          </Column>
          <Column>

             <Label title={"Move Deg"}>
                <Input
                  id={"MinMoveDeg"}
                  style={{ width: "45%", float: "left" }}
                  value={track_move_deg}
                  onChange= {this.onTrackUpdateText}
                  onKeyDown= {this.onTrackKeyText}
                />
              </Label>


          </Column>
          <Column>

             <Label title={"Timeout"}>
                <Input
                  id={"LostTargetTime"}
                  style={{ width: "45%", float: "left" }}
                  value={track_reset_time}
                  onChange= {this.onTrackUpdateText}
                  onKeyDown= {this.onTrackKeyText}
                />
              </Label>



          </Column>
          </Columns>

                  <SliderAdjustment
                            title={"Move Sensitivity"}
                            msgType={"std_msgs/Float32"}
                            adjustment={move_ratio}
                            topic={namespace + "/set_track_move_ratio"}
                            scaled={0.01}
                            min={0}
                            max={100}
                            disabled={false}
                            tooltip={"Sets target confidence threshold filter"}
                            unit={"%"}
                    />


 </div>


 <div style={{ borderTop: "1px solid #777777", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

        <Columns>
            <Column>

                      <div hiddent={has_targets === false}>
                        <ButtonMenu>
                          <Button onClick={() => window.open("/ai_detectors_mgr", "_blank")}>{"Open Detectors"}</Button>
                        </ButtonMenu>

                      </div>
            </Column>

            <Column>
                    <ButtonMenu>
                      <Button onClick={() => window.open("/ai_models_mgr", "_blank")}>{"Open Models"}</Button>
                    </ButtonMenu>          
            </Column>
        </Columns>




            </React.Fragment>
        )
  
}



  onUpdateInputControlNameValue(event, name, index) {
    const value = event.target.value
    var stab_control_values = this.state.stab_control_values
    stab_control_values[index] = value
    this.setState({ stab_control_values: stab_control_values })
    document.getElementById(name).style.color = Styles.vars.colors.red
    //this.render()
  }

  onKeySaveInputControlNameValue(event, name, index) {
    const namespace = this.getNamespace() + '/set_stab_control_value'

    if(event.key === 'Enter'){
      const value = event.target.value
      const parsed = parseFloat(value)
      if (!Number.isNaN(parsed)) {
        this.props.ros.sendUpdateFloatMsg(namespace,name,parsed)
      }
      else {
        const stab_control_values = this.state.status_msg.stab_control_values
        this.setState({ stab_control_values: stab_control_values })
      }
      document.getElementById(name).style.color = Styles.vars.colors.black
    }
  }



    
    renderStabControlValues(name, value, index) {
    return (

      <React.Fragment>

        <Label title={name}>
          <Input id={name} 
              style={{ width: "45%", float: "left" }}
              value={value} 
              onChange={(event) => this.onUpdateInputControlNameValue(event,name,index)} 
              onKeyDown= {(event) => this.onKeySaveInputControlNameValue(event,name,index)} />
        </Label>
      </React.Fragment>
    )
  }





  renderStabControls() {
    const namespace = this.getNamespace()
    const baseNamespace = this.getBaseNamespace()
    const status_msg = this.state.status_msg

    const available_sources = status_msg.available_stab_source_namespaces
    var sources_names = available_sources
    sources_names = removeStringFromMenuNames(sources_names,baseNamespace + '/')
    sources_names = removeStringFromMenuNames(sources_names,'navposes/')
    sources_names = removeStringFromMenuNames(sources_names,'navpose/')
    sources_names = removeStringFromMenuNames(sources_names,'npx/')
    const sources_menu = createMenuListFromStrLists(available_sources, sources_names, ['None'], [], 'None Available')
    const selected_source = status_msg.selected_stab_source

    const available_processes = status_msg.available_stab_processes
    const processes_menu = createMenuListFromStrLists(available_processes, available_processes, ['None'], [], 'None Available')
    const selected_process = status_msg.selected_stab_process

    var stab_update_rate = this.state.stab_update_rate
    if (stab_update_rate == null) { stab_update_rate = status_msg.stab_update_rate }

    var stab_num_avg = this.state.stab_num_avg
    if (stab_num_avg == null) { stab_num_avg = status_msg.stab_num_avg }

    var stab_reset_time_sec = this.state.stab_reset_time_sec
    if (stab_reset_time_sec == null) { stab_reset_time_sec = status_msg.stab_reset_time_sec }

    const show_settings = this.state.stab_show_settings
    const stab_control_names = this.state.stab_control_names
    const stab_control_values = this.state.stab_control_values

    //const servo_max_speed_dps = status_msg.servo_max_speed_dps
    const stab_sv_min_speed_ratio = status_msg.stab_sv_min_speed_ratio
    const stab_sv_max_speed_ratio = status_msg.stab_sv_max_speed_ratio

    const pan_deg = status_msg.pan_deg
    const tilt_deg = status_msg.tilt_deg
    const pan_goal = status_msg.pan_goal
    const tilt_goal = status_msg.tilt_goal
    const pan_deg_per_sec = status_msg.pan_deg_per_sec
    const tilt_deg_per_sec = status_msg.tilt_deg_per_sec


    const stab_pan_pos = status_msg.stab_pan_pos
    const stab_tilt_pos = status_msg.stab_tilt_pos

    const stab_roll_deg = status_msg.stab_roll_deg
    const stab_roll_dps = status_msg.stab_roll_dps
    const stab_pitch_deg = status_msg.stab_pitch_deg
    const stab_pitch_dps = status_msg.stab_pitch_dps
    const stab_heading_deg = status_msg.stab_heading_deg
    const stab_heading_dps = status_msg.stab_heading_dps
    const stab_pan_deg = status_msg.stab_pan_deg
    const stab_pan_adj = status_msg.stab_pan_adj
    const stab_tilt_deg = status_msg.stab_tilt_deg
    const stab_tilt_adj = status_msg.stab_tilt_adj
    const stab_pan_goal = status_msg.stab_pan_goal
    const stab_pan_dps = status_msg.stab_pan_dps
    const stab_tilt_goal = status_msg.stab_tilt_goal
    const stab_tilt_dps = status_msg.stab_tilt_dps

    const stab_pan_pos_rate = status_msg.stab_pan_pos_rate
    const stab_pan_vel_rate = status_msg.stab_pan_vel_rate
    const stab_tilt_pos_rate = status_msg.stab_tilt_pos_rate
    const stab_tilt_vel_rate = status_msg.stab_tilt_vel_rate



    return (
      <React.Fragment>

<div style={{ borderTop: "1px solid #777777", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

        <Label title={""}>
          <div style={{ display: "inline-block", width: "45%", float: "left" }}>{"Ready"}</div>
          <div style={{ display: "inline-block", width: "45%", float: "left" }}>{"Running"}</div>
        </Label>


        <Label title={"Pan"}>
          <div style={{ width: "45%", float: "left" }}>
          <BooleanIndicator value={this.state.stabPanReady} />
          </div>
          <div style={{ width: "45%", float: "left" }}>
           <BooleanIndicator value={this.state.stabPanRunning} />
           </div>
        </Label>

        <Label title={"Tilt"}>
          <div style={{ width: "45%", float: "left" }}>
          <BooleanIndicator value={this.state.stabTiltReady}  />
          </div>
          <div style={{ width: "45%", float: "left" }}>
           <BooleanIndicator value={this.state.stabTiltRunning} />
           </div>
        </Label>
     
          <Label title={""} style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
            <div style={{ display: "inline-block", width: "45%", float: "left" }}>{"Pan"}</div>
            <div style={{ display: "inline-block", width: "45%", float: "left" }}>{"Tilt"}</div>
          </Label>

           <Label title={"Stab Position"}>
            <Input disabled style={{ width: "45%", float: "left" }} value={round(stab_pan_pos, 2)} />
            <Input disabled style={{ width: "45%" }} value={round(stab_tilt_pos, 2)} />
          </Label>

<div style={{ borderTop: "1px solid #777777", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

            <div style={{ display: 'flex' }} >
                <div style={{ width: '60%' }} >

                        <Label title="Show Stab Settings">
                            <Toggle
                              checked={(show_settings)}
                              onClick={() => onChangeSwitchStateValue.bind(this)("stab_show_settings",show_settings)}>
                            </Toggle>
                        </Label>

                </div>

                <div style={{ width: '40%' }}>
                </div>

          </div>
          
          <div hidden={(show_settings === false)}>

                

                      <Label title={'Select Source'}>
                        <Select
                          id="set_stab_source"
                          onChange={(e) => this.props.ros.sendStringMsg(namespace + "/set_stab_source", e.target.value)}
                          value={selected_source}
                        >
                          {sources_menu}
                        </Select>
                      </Label>

                      <Label title={'Select Process'}>
                        <Select
                          id="set_stab_process"
                          onChange={(e) => this.props.ros.sendStringMsg(namespace + "/set_stab_process", e.target.value)}
                          value={selected_process}
                        >
                          {processes_menu}
                        </Select>
                      </Label>



                <div style={{ borderTop: "1px solid #000000", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>


                <Label title={"Update Rate"}>
                  <Input
                    id={"StabUpdateRate"}
                    style={{ width: "45%", float: "left" }}
                    value={stab_update_rate}
                    onChange={this.onStabUpdateText}
                    onKeyDown={this.onStabKeyText}
                  />
                </Label>

                <Label title={"Num Avg"}>
                  <Input
                    id={"StabNumAvg"}
                    style={{ width: "45%", float: "left" }}
                    value={stab_num_avg}
                    onChange={this.onStabUpdateText}
                    onKeyDown={this.onStabKeyText}
                  />
                </Label>


                  <div>
                      {/* Map over the stab control names array */}
                      {stab_control_names.map((name, index) => (
                        this.renderStabControlValues(name, stab_control_values[index], index)
                      ))}
                    </div>

                  <RangeAdjustment
                    title="Min Max Speed"
                    min={stab_sv_min_speed_ratio}
                    max={stab_sv_max_speed_ratio}
                    topic={namespace + "/set_stab_max_speed_ratios"}
                    tooltip={"Sets stabilize min max move ratios"}
                    noTextBox={true}
                  />        

              <ButtonMenu>
                <Button onClick={() => this.props.ros.sendTriggerMsg(namespace + "/reload_stab_processes")}>{"Reload Processes"}</Button>
              </ButtonMenu>

 


          <div style={{ borderTop: "1px solid #777777", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>


          <Label title={""}>
            <div style={{ display: "inline-block", width: "45%", float: "left" }}>{"Deg"}</div>
            <div style={{ display: "inline-block", width: "45%", float: "left" }}>{"DPS"}</div>
          </Label>


          <Label title={"Pan"}>
            <Input disabled style={{ width: "45%", float: "left" }} value={round(pan_deg, 2)} />
            <Input disabled style={{ width: "45%" }} value={round(pan_deg_per_sec, 2)} />
          </Label>

          <Label title={"Tilt"}>
            <Input disabled style={{ width: "45%", float: "left" }} value={round(tilt_deg, 2)} />
            <Input disabled style={{ width: "45%" }} value={round(tilt_deg_per_sec, 2)} />
          </Label>

          <Label title={""}>
            <div style={{ display: "inline-block", width: "30%", float: "left" }}>{"Roll"}</div>
            <div style={{ display: "inline-block", width: "30%", float: "center" }}>{"Pitch"}</div>
            <div style={{ display: "inline-block", width: "30%", float: "right" }}>{"Yaw"}</div>
          </Label>

          <Label title={"Nav"}>
            <Input disabled style={{ width: "30%", float: "left" }} value={round(stab_roll_deg, 2)} />
            <Input disabled style={{ width: "30%", float: "center" }} value={round(stab_pitch_deg, 2)} />
            <Input disabled style={{ width: "30%", float: "right" }} value={round(stab_heading_deg, 2)} />
          </Label>

          <Label title={"Stab Updates"}>
            <div style={{ display: "inline-block", width: "30%", float: "left" }}>{"Adj"}</div>
            <div style={{ display: "inline-block", width: "30%", float: "center" }}>{"Goal"}</div>
            <div style={{ display: "inline-block", width: "30%", float: "right" }}>{"Speed"}</div>
          </Label>

          <Label title={"Pan"}>
            <Input disabled style={{ width: "30%", float: "left" }} value={round(stab_pan_adj, 2)} />
            <Input disabled style={{ width: "30%", float: "center" }} value={round(stab_pan_goal, 2)} />
            <Input disabled style={{ width: "30%", float: "right" }} value={round(stab_pan_dps, 2)} />
          </Label>

          <Label title={"Tilt"}>
            <Input disabled style={{ width: "30%", float: "left" }} value={round(stab_tilt_adj, 2)} />
            <Input disabled style={{ width: "30%", float: "center" }} value={round(stab_tilt_goal, 2)} />
            <Input disabled style={{ width: "30%", float: "right" }} value={round(stab_tilt_dps, 2)} />
          </Label>

          <Label title={"Stab Rates"}>
            <div style={{ display: "inline-block", width: "45%", float: "left" }}>{"Pos"}</div>
            <div style={{ display: "inline-block", width: "45%", float: "left" }}>{"Speed"}</div>
          </Label>


          <Label title={"Pan"}>
            <Input disabled style={{ width: "45%", float: "left" }} value={round(stab_pan_pos_rate, 2)} />
            <Input disabled style={{ width: "45%" }} value={round(stab_pan_vel_rate, 2)} />
          </Label>

          <Label title={"Tilt"}>
            <Input disabled style={{ width: "45%", float: "left" }} value={round(stab_tilt_pos_rate, 2)} />
            <Input disabled style={{ width: "45%" }} value={round(stab_tilt_vel_rate, 2)} />
          </Label>

         </div>

      </React.Fragment>
    )
  }

  render() {
    const make_section = (this.props.make_section !== undefined)? this.props.make_section : true

    const status_msg = this.state.status_msg
    if (status_msg == null){
      return (
        <Columns>
        <Column>
       
        </Column>
        </Columns>
      )


    }
    else if (make_section === false){

      return (

          <React.Fragment>

              { this.renderSVAutoControls()}


          </React.Fragment>
      )
    }
    else {
      return (

          <Section title={(this.props.title !== undefined) ? this.props.title : ""}>


              {this.renderSVAutoControls()}


        </Section>
     )
   }

  }


}

export default NepiAppSVAutoControls
