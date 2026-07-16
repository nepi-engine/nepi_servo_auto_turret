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
// One SVX device = one servo. Controls are drawn conditionally on the Figure 4
// capability flags reported by the device. All field/topic names match the
// DeviceSVXStatus msg (Figure 3), SVXCapabilitiesQuery srv (Figure 4), and the
// Figure 5 control topics. Editable inputs follow the authoritative RUI pattern.

import React, { Component } from "react"
import { observer, inject } from "mobx-react"
import Toggle from "react-toggle"

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

      speedMax: 0.0,

      show_controls: false,

      homePos : null,
      softStopMin : null,
      softStopMax : null,
      gotoPos : null,

      statusListener: null,

    }


    this.onUpdateText = this.onUpdateText.bind(this)
    this.onKeyText = this.onKeyText.bind(this)

    this.renderControlPanel = this.renderControlPanel.bind(this)
    this.renderDeviceIF = this.renderDeviceIF.bind(this)

    this.updateStatusListener = this.updateStatusListener.bind(this)
    this.statusListener = this.statusListener.bind(this)
  }


  // Callback for handling ROS DeviceSVXStatus messages
  statusListener(message) {
    const last_status_msg = this.state.status_msg
    this.setState({
      status_msg: message
    })

    const speedMax = message.speed_max_dps
    const homePos = message.home_pos_deg
    const softStopMin = message.min_softstop_deg
    const softStopMax = message.max_softstop_deg

    var needs_update = false
    if (last_status_msg == null){
      needs_update = true
    }
    else {
       needs_update = (
          speedMax !== last_status_msg.speed_max_dps ||
          homePos !== last_status_msg.home_pos_deg ||
          softStopMin !== last_status_msg.min_softstop_deg ||
          softStopMax !== last_status_msg.max_softstop_deg
      )
    }
    if (needs_update === true){
      this.setState({
          speedMax: speedMax,
          homePos : round(message.home_pos_deg, 1),
          softStopMin : round(message.min_softstop_deg, 1),
          softStopMax : round(message.max_softstop_deg, 1)
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
                    softStopMin : null,
                    softStopMax : null,
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
  }


  onUpdateText(e) {
    if (e.target.id === "SVXSoftStopMin")
    {
      const el = document.getElementById("SVXSoftStopMin")
      setElementStyleModified(el)
      this.setState({softStopMin: e.target.value})
    }
    else if (e.target.id === "SVXSoftStopMax")
    {
      const el = document.getElementById("SVXSoftStopMax")
      setElementStyleModified(el)
      this.setState({softStopMax: e.target.value})
    }
    else if (e.target.id === "SVXHomePos")
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
    else if (e.target.id === "SVXMaxSpeed")
    {
      const el = document.getElementById("SVXMaxSpeed")
      setElementStyleModified(el)
      this.setState({speedMax: e.target.value})
    }
  }

  onKeyText(e) {
    const { sendFloatMsg } = this.props.ros
    const namespace = (this.props.namespace !== undefined) ? this.props.namespace : 'None'

    if(e.key === 'Enter'){

      if ((e.target.id === "SVXSoftStopMin") || (e.target.id === "SVXSoftStopMax"))
      {
        const minEl = document.getElementById("SVXSoftStopMin")
        const maxEl = document.getElementById("SVXSoftStopMax")
        clearElementStyleModified(minEl)
        clearElementStyleModified(maxEl)
        // set_soft_limits control: ServoLimits msg {min_deg, max_deg}
        this.props.ros.publishMessage({
          name: namespace + "/set_soft_limits",
          messageType: "nepi_interfaces/ServoLimits",
          data: {"min_deg": Number(minEl.value), "max_deg": Number(maxEl.value)},
          noPrefix: true
        })
      }
      else if (e.target.id === "SVXHomePos")
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
      else if (e.target.id === "SVXMaxSpeed")
      {
        const el = document.getElementById("SVXMaxSpeed")
        clearElementStyleModified(el)
        sendFloatMsg(namespace + "/set_speed_max_dps", el.value)
      }
    }
  }


  renderControlPanel() {

    const { sendBoolMsg, sendTriggerMsg } = this.props.ros
    const namespace = this.props.namespace ? this.props.namespace : 'None'
    const status_msg = this.state.status_msg

    const devices = this.props.ros.svxDevices
    var has_limit_control = false
    var has_homing = false
    var has_set_home = false
    const devicesList = Object.keys(devices)
    if (devicesList.indexOf(namespace) !== -1){
      const capabilities = devices[namespace]
      has_limit_control = capabilities && (capabilities.has_limit_control === true)
      has_homing = capabilities && (capabilities.has_homing === true)
      has_set_home = capabilities && (capabilities.has_set_home === true)
    }

    const reverseEnabled = status_msg.reverse_enabled

    const softStopMin = this.state.softStopMin
    const softStopMax = this.state.softStopMax
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

                    <Label title={"Reverse Control"}>
                      <Toggle checked={reverseEnabled} onClick={() => sendBoolMsg.bind(this)(namespace + "/set_reverse_enable",!reverseEnabled)} />
                    </Label>


                    <div hidden={(has_limit_control === false)}>

                            <Label title={"Soft Limit Min"}>
                              <Input
                                disabled={!has_limit_control}
                                id={"SVXSoftStopMin"}
                                style={{ width: "45%", float: "left" }}
                                value={softStopMin}
                                onChange= {this.onUpdateText}
                                onKeyDown= {this.onKeyText}
                              />
                            </Label>
                            <Label title={"Soft Limit Max"}>
                              <Input
                                disabled={!has_limit_control}
                                id={"SVXSoftStopMax"}
                                style={{ width: "45%", float: "left" }}
                                value={softStopMax}
                                onChange= {this.onUpdateText}
                                onKeyDown= {this.onKeyText}
                              />
                            </Label>

                    </div>


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


  renderDeviceIF() {
    const { sendTriggerMsg, sendIntMsg } = this.props.ros
    const namespace = (this.props.namespace !== undefined) ? this.props.namespace : null
    const status_msg = this.state.status_msg

    const devices = this.props.ros.svxDevices
    var has_abs_pos = false
    var has_goto_control = false
    var has_speed_control = false
    var has_stop_control = false
    var has_spin_control = false
    var has_homing = false
    const devicesList = Object.keys(devices)
    if (devicesList.indexOf(namespace) !== -1){
      const capabilities = devices[namespace]
      has_abs_pos = capabilities && (capabilities.has_absolute_positioning === true)
      has_goto_control = capabilities && (capabilities.has_goto_control === true)
      has_speed_control = capabilities && (capabilities.has_adjustable_speed === true)
      has_stop_control = capabilities && (capabilities.has_stop_control === true)
      has_spin_control = capabilities && (capabilities.has_spin_control === true)
      has_homing = capabilities && (capabilities.has_homing === true)
    }

    const position_now = status_msg.position_now_deg
    const position_goal = status_msg.position_goal_deg
    const positionNowClean = position_now + .001
    const positionGoalClean = position_goal + .001

    const speedRatio = status_msg.speed_ratio
    const spinDirection = status_msg.spin_direction

    return (
      <React.Fragment>

          <ButtonMenu>
            <Button disabled={!has_stop_control} onClick={() => sendTriggerMsg(namespace + "/stop_moving")}>{"STOP"}</Button>
            <Button disabled={!has_homing} onClick={() => sendTriggerMsg(namespace + "/go_home")}>{"GO HOME"}</Button>
          </ButtonMenu>


          <div hidden={(has_goto_control === false && has_abs_pos === false)}>

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

              <Label title={"Current Position"}>
                <Input
                  disabled
                  style={{ width: "45%", float: "left" }}
                  value={round(positionNowClean, 2)}
                />
              </Label>

              <Label title={"Goal Position"}>
                <Input
                  disabled
                  style={{ width: "45%", float: "left" }}
                  value={round(positionGoalClean, 2)}
                />
              </Label>

          </div>


          <div hidden={(has_speed_control === false)}>

            <Label title={"Max Speed (dps)"}>
                <Input
                  id={"SVXMaxSpeed"}
                  style={{ width: "45%" }}
                  value={this.state.speedMax}
                  onChange= {this.onUpdateText}
                  onKeyDown= {this.onKeyText}
                />
            </Label>

            <SliderAdjustment
              disabled={!has_speed_control}
              title={"Speed"}
              msgType={"std_msgs/Float32"}
              adjustment={speedRatio}
              topic={namespace + "/set_speed_ratio"}
              scaled={0.01}
              min={0}
              max={100}
              tooltip={"Speed as a percentage (0%=min, 100%=max)"}
              unit={"%"}
            />

          </div>


          <div hidden={(has_spin_control === false)}>

            <Label title={"Spin Direction"}>
              <Toggle
                checked={spinDirection >= 0}
                onClick={() => sendIntMsg(namespace + "/set_spin_direction", (spinDirection >= 0) ? -1 : 1)}>
              </Toggle>
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
