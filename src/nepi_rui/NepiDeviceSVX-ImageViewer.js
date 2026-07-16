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

// SVX (servo) image viewer panel.
// One SVX device = one servo, so there is a single position slider (goto_ratio)
// drawn only when the device reports absolute positioning. All field/topic names
// match the DeviceSVXStatus msg (Figure 3), SVXCapabilitiesQuery srv (Figure 4),
// and the Figure 5 control topics.

import React, { Component } from "react"

import { observer, inject } from "mobx-react"

import Button, { ButtonMenu } from "./Button"

import { Column, Columns } from "./Columns"


import { SliderAdjustment } from "./AdjustmentWidgets"

import NepiIFImageViewerSelector from "./Nepi_IF_ImageViewerSelector"
import NepiIFImageViewersSelector from "./Nepi_IF_ImageViewerSelector"


@inject("ros")
@observer
class NepiDeviceSVXImageViewer extends Component {
  constructor(props) {
    super(props)

    this.state = {

      namespace: null,

      status_msg: null,
      statusListener: null
    }


    this.getNamespace = this.getNamespace.bind(this)
    this.updateStatusListener = this.updateStatusListener.bind(this)
    this.statusListener = this.statusListener.bind(this)
  }


  getNamespace(){
    const { namespacePrefix, deviceId} = this.props.ros
    var namespace = 'None'
    if (namespacePrefix !== null && deviceId !== null){
      if (this.props.namespace !== undefined){
        namespace = this.props.namespace
      }
      else{
        namespace = "/" + namespacePrefix + "/" + deviceId + "/" + this.state.appName
      }
    }
    return namespace
  }

  // Callback for handling ROS DeviceSVXStatus messages
  statusListener(message) {
    this.setState({
      status_msg: message
    })
  }

  // Function for configuring and subscribing to Status
  updateStatusListener() {
    const { namespace } = this.props
    if (this.state.statusListener !== null) {
      this.state.statusListener.unsubscribe()
       this.setState({ status_msg: null, statusListener: null})
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
  const { namespace } = this.props
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




  render() {
    const { svxDevices, sendTriggerMsg } = this.props.ros
    const namespace = (this.props.namespace !== null) ? this.props.namespace : 'None'
    const status_msg = this.state.status_msg

    const use_images_selector = (this.props.use_images_selector !== undefined) ? this.props.use_images_selector : false
    const show_save_controls = (this.props.show_save_controls !== undefined) ? this.props.show_save_controls : false
    const show_image_controls = (this.props.show_image_controls !== undefined) ? this.props.show_image_controls : false
    const mouse_event_topic = (this.props.mouse_event_topic !== undefined) ? this.props.mouse_event_topic : null



    var positionGoalRatio = 0.5
    if (status_msg != null){
      positionGoalRatio = status_msg.position_goal_ratio
    }


    const svxDevicesList = Object.keys(svxDevices)
    var has_abs_pos = false
    var has_stop_control = false
    if (svxDevicesList.indexOf(namespace) !== -1){
      const svx_caps = svxDevices[namespace]
      has_abs_pos = svx_caps && (svx_caps.has_absolute_positioning === true)
      has_stop_control = svx_caps && (svx_caps.has_stop_control === true)
    }

    const show_sv_controls = (has_abs_pos === true)


    return (

        <React.Fragment >

        <Columns>
          <Column equalWidth = {false} >

          <div id={'svxImageViewer'}>

            {(use_images_selector === true) ?
              <NepiIFImageViewersSelector

                hideQualitySelector={true}
                show_save_controls={show_save_controls}
                show_image_controls={show_image_controls}
                mouse_event_topic={mouse_event_topic}
              />
              :
                  <NepiIFImageViewerSelector
                    id={'svxImageViewer'}
                    hideQualitySelector={true}
                    show_save_controls={show_save_controls}
                    show_image_controls={show_image_controls}
                    mouse_event_topic={mouse_event_topic}

                  />
            }

        </div>

          </Column>
        </Columns>



              <div hidden={show_sv_controls === false}>

                  <SliderAdjustment
                    title={"Position"}
                    msgType={"std_msgs/Float32"}
                    adjustment={positionGoalRatio}
                    topic={namespace + "/goto_ratio"}
                    scaled={0.01}
                    min={0}
                    max={100}
                    tooltip={"Position as a percentage (0%=min, 100%=max)"}
                    unit={"%"}
                    noTextBox={true}
                    noLabel={true}
                  />
              </div>


              <div hidden={has_stop_control === false}>
                <ButtonMenu>
                    <Button onClick={() => sendTriggerMsg(namespace + "/stop_moving")}>{"STOP"}</Button>
                </ButtonMenu>
              </div>

  </React.Fragment>




    )
  }


}

export default NepiDeviceSVXImageViewer
