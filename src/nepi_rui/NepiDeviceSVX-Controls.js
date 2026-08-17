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

// SVX (servo) controls panel.
// One SVX device = one servo. Controls are drawn conditionally on the capability
// flags reported by the device (SVXCapabilitiesQuery). A servo is EITHER positional
// (has_absolute_positioning) OR continuous-rotation (has_spin_control); the bottom
// slider adapts to whichever the device reports. Soft limits were removed -- the
// hardstops are the single clamp range -- so there are no soft-limit inputs here.

import React, { Component } from "react"
import { observer, inject } from "mobx-react"
import Toggle from "react-toggle"
import Slider from "rc-slider"

import Section from "./Section"
import { Columns, Column } from "./Columns"
import { SliderAdjustment } from "./AdjustmentWidgets"
import Label from "./Label"
import Input from "./Input"
import Styles from "./Styles"
import Button, { ButtonMenu } from "./Button"

import {setElementStyleModified, clearElementStyleModified, onChangeSwitchStateValue, round} from "./Utilities"

import NepiIFConfig from "./Nepi_IF_Config"

@inject("ros")
@observer

// Component that contains the SVX controls
class NepiDeviceSVXControls extends Component {
  constructor(props) {
    super(props)

    this.state = {

      namespace : null,
      status_msg: null,

      show_controls: false,

      // Continuous-spinning mode toggle (UI-local). When on, the controls switch
      // from degree position to direction+speed; the bipolar slider publishes to the
      // existing set_spin_direction / set_speed_ratio topics.
      spinning_mode: false,

      homePos : null,
      gotoPos : null,

      // Position readout unit toggle: false = degrees, true = PWM microseconds.
      show_pwm : false,
      // pulse_min_us / pulse_max_us read from the device's own settings (node-owned).
      // Null until the settings arrive; the PWM toggle stays disabled while null.
      pulseMin : null,
      pulseMax : null,

      // Local handle position for the continuous-mode bipolar slider while dragging.
      bipolar : null,

      statusListener: null,
      settingsTopic: null,
      settingsListener: null,

    }


    this.onUpdateText = this.onUpdateText.bind(this)
    this.onKeyText = this.onKeyText.bind(this)

    this.renderControlPanel = this.renderControlPanel.bind(this)
    this.renderSettings = this.renderSettings.bind(this)
    this.renderDeviceIF = this.renderDeviceIF.bind(this)

    this.updateStatusListener = this.updateStatusListener.bind(this)
    this.statusListener = this.statusListener.bind(this)
    this.updateSettingsListener = this.updateSettingsListener.bind(this)
    this.settingsListener = this.settingsListener.bind(this)

    this.degToPwm = this.degToPwm.bind(this)
    this.pwmAvailable = this.pwmAvailable.bind(this)
    this.posDisplay = this.posDisplay.bind(this)
    this.onBipolarChange = this.onBipolarChange.bind(this)
  }


  // Callback for handling ROS DeviceSVXStatus messages
  statusListener(message) {
    const last_status_msg = this.state.status_msg
    this.setState({
      status_msg: message
    })

    // Subscribe to the device's settings the first time we learn where they live,
    // and re-subscribe if the settings topic ever changes. pulse_min_us/pulse_max_us
    // are node-owned settings and are what the PWM readout needs.
    if (message.settings_topic && message.settings_topic !== this.state.settingsTopic) {
      this.updateSettingsListener(message.settings_topic)
    }

    const homePos = message.home_pos_deg

    var needs_update = false
    if (last_status_msg == null){
      needs_update = true
    }
    else {
       needs_update = (
          homePos !== last_status_msg.home_pos_deg
      )
    }
    if (needs_update === true){
      this.setState({
          homePos : round(message.home_pos_deg, 1)
      })
    }

  }

  // Function for configuring and subscribing to Status
  updateStatusListener() {
    const namespace = (this.props.namespace !== undefined) ? this.props.namespace : null
    if (this.state.statusListener != null) {
      this.state.statusListener.unsubscribe()
       this.setState({ status_msg: null, statusListener: null})
      this.setState({homePos : null,
                    gotoPos : null
      })
    }
    if (namespace != null && namespace !== 'None'){
        var statusListener = this.props.ros.setupSVXStatusListener(
              namespace,
              this.statusListener
            )
      this.setState({ statusListener: statusListener})
    }
    this.setState({ namespace: namespace})

}

  // Subscribe to the device's SettingsStatus so we can read pulse_min_us/pulse_max_us
  // for the degrees<->PWM readout conversion. Those are per-servo, node-owned settings
  // (e.g. the Maestro exposes them); a servo that does not expose them simply leaves
  // the PWM toggle disabled.
  updateSettingsListener(settings_topic) {
    if (this.state.settingsListener != null) {
      this.state.settingsListener.unsubscribe()
      this.setState({ settingsListener: null, pulseMin: null, pulseMax: null })
    }
    if (settings_topic) {
      // The SettingsStatus is published at "<settings_topic>/status" (matches
      // Nepi_IF_Settings.js in the base RUI).
      const settingsListener = this.props.ros.setupSettingsStatusListener(
        settings_topic + '/status',
        this.settingsListener
      )
      this.setState({ settingsListener: settingsListener })
    }
    this.setState({ settingsTopic: settings_topic })
  }

  settingsListener(message) {
    const list = (message && message.settings_list) ? message.settings_list : []
    var pmin = null
    var pmax = null
    for (let i = 0; i < list.length; i++) {
      const s = list[i]
      if (s.name_str === "pulse_min_us") { pmin = Number(s.value_str) }
      else if (s.name_str === "pulse_max_us") { pmax = Number(s.value_str) }
    }
    if (pmin !== this.state.pulseMin || pmax !== this.state.pulseMax) {
      this.setState({ pulseMin: pmin, pulseMax: pmax })
    }
  }

// Lifecycle method called when component updates.
// Used to track changes in the topic
componentDidUpdate(prevProps, prevState, snapshot) {
  const namespace = (this.props.namespace !== undefined) ? this.props.namespace : null
   if (namespace !== this.state.namespace){
      this.updateStatusListener()
  }
}

  componentDidMount() {
    this.updateStatusListener()
    }


  // Lifecycle method called just before the component unmounts.
  componentWillUnmount() {
    if (this.state.statusListener) {
      this.state.statusListener.unsubscribe()
      this.setState({statusListener : null})
    }
    if (this.state.settingsListener) {
      this.state.settingsListener.unsubscribe()
      this.setState({settingsListener : null})
    }
  }


  // Linear degrees -> pulse-width (microseconds). Maps the device's declared travel
  // (min/max_softstop_deg, which now carry the hardstop range) onto its pulse
  // endpoints. Returns null when the pulse settings are not (yet) available.
  degToPwm(deg) {
    const status = this.state.status_msg
    const pmin = this.state.pulseMin
    const pmax = this.state.pulseMax
    if (status == null || pmin == null || pmax == null || isNaN(pmin) || isNaN(pmax)) {
      return null
    }
    const dmin = status.min_softstop_deg
    const dmax = status.max_softstop_deg
    const span = dmax - dmin
    if (span === 0) { return pmin }
    const t = (deg - dmin) / span
    return pmin + t * (pmax - pmin)
  }

  pwmAvailable() {
    return (this.state.pulseMin != null && this.state.pulseMax != null &&
            !isNaN(this.state.pulseMin) && !isNaN(this.state.pulseMax))
  }

  // Format a position for a read-only field, honoring the degrees/PWM toggle.
  posDisplay(deg) {
    if (this.state.show_pwm === true && this.pwmAvailable()) {
      return round(this.degToPwm(deg), 0).toString() + " µs"
    }
    return round(deg, 2).toString() + "°"
  }


  onUpdateText(e) {
    if (e.target.id === "SVXHomePos")
    {
      const el = document.getElementById("SVXHomePos")
      setElementStyleModified(el)
      this.setState({homePos: e.target.value})
    }
    else if (e.target.id === "SVXGoto")
    {
      const el = document.getElementById("SVXGoto")
      setElementStyleModified(el)
      this.setState({gotoPos: e.target.value})
    }
  }

  onKeyText(e) {
    const { sendFloatMsg } = this.props.ros
    const namespace = (this.props.namespace !== undefined) ? this.props.namespace : 'None'

    if(e.key === 'Enter'){

      if (e.target.id === "SVXHomePos")
      {
        const el = document.getElementById("SVXHomePos")
        clearElementStyleModified(el)
        sendFloatMsg(namespace + "/set_home_position", el.value)
      }
      else if (e.target.id === "SVXGoto")
      {
        const el = document.getElementById("SVXGoto")
        clearElementStyleModified(el)
        sendFloatMsg(namespace + "/goto_position", el.value)
        this.setState({gotoPos: null})
      }
    }
  }

  // Continuous mode: one bipolar slider (-100..0..+100). Sign is the spin direction,
  // magnitude is the speed ratio. Publishes to the two existing control topics.
  onBipolarChange(value) {
    const { sendIntMsg, sendFloatMsg } = this.props.ros
    const namespace = (this.props.namespace !== undefined) ? this.props.namespace : 'None'
    const v = Number(value)
    const dir = (v >= 0) ? 1 : -1
    const ratio = Math.abs(v) / 100.0
    sendIntMsg(namespace + "/set_spin_direction", dir)
    sendFloatMsg(namespace + "/set_speed_ratio", ratio)
  }


  renderControlPanel() {

    const { sendTriggerMsg } = this.props.ros
    const namespace = this.props.namespace ? this.props.namespace : 'None'
    const status_msg = this.state.status_msg

    const devices = this.props.ros.svxDevices
    var has_homing = false
    var has_set_home = false
    const devicesList = Object.keys(devices)
    if (devicesList.indexOf(namespace) !== -1){
      const capabilities = devices[namespace]
      has_homing = capabilities && (capabilities.has_homing === true)
      has_set_home = capabilities && (capabilities.has_set_home === true)
    }

    const homePos = this.state.homePos

    const show_controls =  this.state.show_controls

    return (
      <React.Fragment>

            <Columns>
              <Column>

                  <Label title="Show Controls">
                      <Toggle
                        checked={show_controls===true}
                        onClick={() => onChangeSwitchStateValue.bind(this)("show_controls",show_controls)}>
                      </Toggle>
                  </Label>

                </Column>
                <Column>

                </Column>
              </Columns>


              <div hidden={(show_controls===false)}>

              <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>


                    <div hidden={(has_homing === false)}>

                    <Label title={"Home Position"}>
                      <Input
                        disabled={!has_homing}
                        id={"SVXHomePos"}
                        style={{ width: "45%", float: "left" }}
                        value={homePos}
                        onChange= {this.onUpdateText}
                        onKeyDown= {this.onKeyText}
                      />
                    </Label>

                    <ButtonMenu>
                      <Button disabled={!has_set_home} onClick={() => sendTriggerMsg(namespace + "/set_home_position_here")}>{"Set Home Here"}</Button>
                    </ButtonMenu>

                  </div>


                  <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

                  <ButtonMenu>
                    <Button onClick={() => sendTriggerMsg(namespace + "/reset_device")}>{"Reset Device"}</Button>
                  </ButtonMenu>

              </div>


        </React.Fragment>
      )
  }


  // Reverse Control lives with the device settings (relocated out of the live
  // controls). It still publishes the same set_reverse_enable topic.
  renderSettings() {
    const { sendBoolMsg } = this.props.ros
    const namespace = this.props.namespace ? this.props.namespace : 'None'
    const status_msg = this.state.status_msg
    if (status_msg == null){
      return null
    }
    const reverseEnabled = status_msg.reverse_enabled

    return (
      <React.Fragment>
        <Label title={"Reverse Control"}>
          <Toggle checked={reverseEnabled} onClick={() => sendBoolMsg.bind(this)(namespace + "/set_reverse_enable",!reverseEnabled)} />
        </Label>
      </React.Fragment>
    )
  }


  renderDeviceIF() {
    const { sendTriggerMsg } = this.props.ros
    const namespace = (this.props.namespace !== undefined) ? this.props.namespace : null
    const status_msg = this.state.status_msg

    const devices = this.props.ros.svxDevices
    var has_abs_pos = false
    var has_goto_control = false
    var has_stop_control = false
    var has_homing = false
    const devicesList = Object.keys(devices)
    if (devicesList.indexOf(namespace) !== -1){
      const capabilities = devices[namespace]
      has_abs_pos = capabilities && (capabilities.has_absolute_positioning === true)
      has_goto_control = capabilities && (capabilities.has_goto_control === true)
      has_stop_control = capabilities && (capabilities.has_stop_control === true)
      has_homing = capabilities && (capabilities.has_homing === true)
    }

    const position_now = status_msg.position_now_deg
    const position_goal = status_msg.position_goal_deg
    const positionNowClean = position_now + .001
    const positionGoalClean = position_goal + .001

    // Position slider range = the declared hard-travel bounds (soft limits removed).
    const softStopMin = status_msg.min_softstop_deg
    const softStopMax = status_msg.max_softstop_deg

    // Continuous-mode bipolar value from status (sign = direction, magnitude = speed).
    const spinDirection = status_msg.spin_direction
    const speedRatio = status_msg.speed_ratio
    const bipolarFromStatus = ((spinDirection >= 0) ? 1 : -1) * (speedRatio || 0) * 100
    const bipolarVal = (this.state.bipolar != null) ? this.state.bipolar : bipolarFromStatus

    const pwmAvail = this.pwmAvailable()

    return (
      <React.Fragment>

          <ButtonMenu>
            <Button disabled={!has_stop_control} onClick={() => sendTriggerMsg(namespace + "/stop_moving")}>{"STOP"}</Button>
            <Button disabled={!has_homing} onClick={() => sendTriggerMsg(namespace + "/go_home")}>{"GO HOME"}</Button>
          </ButtonMenu>


          <Label title={"Continuous Spinning"}>
            <Toggle
              checked={this.state.spinning_mode === true}
              onClick={() => this.setState({ spinning_mode: !this.state.spinning_mode })}>
            </Toggle>
          </Label>


          <div hidden={(this.state.spinning_mode === true) || (has_goto_control === false && has_abs_pos === false)}>

              <SliderAdjustment
                disabled={!has_goto_control}
                title={"Position"}
                msgType={"std_msgs/Float32"}
                adjustment={position_goal}
                topic={namespace + "/goto_position"}
                scaled={1}
                min={softStopMin}
                max={softStopMax}
                tooltip={"Commanded position in degrees"}
                unit={"°"}
              />

              <Label title={"GoTo Position (deg)"}>
                <Input
                  disabled={!has_goto_control}
                  id={"SVXGoto"}
                  style={{ width: "45%", float: "left" }}
                  value={this.state.gotoPos}
                  onChange= {this.onUpdateText}
                  onKeyDown= {this.onKeyText}
                />
              </Label>

              <Label title={"Show PWM (µs)"}>
                <Toggle
                  disabled={!pwmAvail}
                  checked={this.state.show_pwm === true && pwmAvail}
                  onClick={() => this.setState({ show_pwm: !this.state.show_pwm })}>
                </Toggle>
              </Label>

              <Label title={"Current Position"}>
                <Input
                  disabled
                  style={{ width: "45%", float: "left" }}
                  value={this.posDisplay(positionNowClean)}
                />
              </Label>

              <Label title={"Goal Position"}>
                <Input
                  disabled
                  style={{ width: "45%", float: "left" }}
                  value={this.posDisplay(positionGoalClean)}
                />
              </Label>

          </div>


          <div hidden={(this.state.spinning_mode === false)}>

            <Label title={"Speed / Direction"}>
              <div style={{ width: "60%", float: "left" }}>
                <Slider
                  min={-100}
                  max={100}
                  value={bipolarVal}
                  onChange={(v) => this.setState({ bipolar: v })}
                  onAfterChange={(v) => { this.onBipolarChange(v); this.setState({ bipolar: null }) }}
                />
              </div>
            </Label>

          </div>


      </React.Fragment>
      )

  }


  render() {
    const make_section = (this.props.make_section !== undefined)? this.props.make_section : true
    const namespace = (this.props.namespace !== undefined) ? this.props.namespace : null
    const show_controls = (this.props.show_controls !== undefined) ? this.props.show_controls : true
    const show_config = (this.props.show_config !== undefined) ? this.props.show_config : true
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

              { (status_msg != null) ? this.renderDeviceIF() : null}
              { (status_msg != null && show_controls === true) ? this.renderControlPanel() : null}
              { (status_msg != null && show_config === true) ? this.renderSettings() : null}
              { (status_msg != null && show_config === true) ?
                <NepiIFConfig
                namespace={namespace}
                show_save_all={true}
                title={"Nepi_IF_Conig"}
                />
              : null }


    </React.Fragment>
      )
    }
    else {
      return (

          <Section title={(this.props.title !== undefined) ? this.props.title : ""}>

              { (status_msg != null) ? this.renderDeviceIF() : null}
              { (status_msg != null && show_controls === true) ? this.renderControlPanel() : null}
              { (status_msg != null && show_config === true) ? this.renderSettings() : null}
              { (status_msg != null && show_config === true) ?
                <NepiIFConfig
                namespace={namespace}
                show_save_all={true}
                title={"Nepi_IF_Conig"}
                />
              : null }
        </Section>
     )
    }
  }
}

export default NepiDeviceSVXControls
