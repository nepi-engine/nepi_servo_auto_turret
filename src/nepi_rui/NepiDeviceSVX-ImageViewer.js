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

//import moment from "moment"
import { observer, inject } from "mobx-react"

//import { TransformWrapper, TransformComponent } from "react-zoom-pan-pinch";

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
      statusListener: null,

      jog_speed_ratio: 0.5
    }


    //this.renderImageViewer = this.renderImageViewer.bind(this)

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

  // Callback for handling ROS Status3DX messages
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
  
// Lifecycle method called when compnent updates.
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



  // Lifecycle method called just before the component umounts.
  // Used to unsubscribe to Status3DX message
  componentWillUnmount() {
    if (this.state.statusListener) {
      this.state.statusListener.unsubscribe()
      this.setState({statusListener : null})
    }
  }




  render() {
    const { svxDevices, onSVXJogPan, onSVXJogTilt, onSVXJogSpeedPan, onSVXJogSpeedTilt, onSVXStop, onSVXPanStop, onSVXTiltStop } = this.props.ros
    const jog_speed_ratio = this.state.jog_speed_ratio
    const namespace = (this.props.namespace !== null) ? this.props.namespace : 'None'
    const status_msg = this.state.status_msg

    const use_images_selector = (this.props.use_images_selector !== undefined) ? this.props.use_images_selector : false
    const show_save_controls = (this.props.show_save_controls !== undefined) ? this.props.show_save_controls : false
    const show_image_controls = (this.props.show_image_controls !== undefined) ? this.props.show_image_controls : false
    const mouse_event_topic = (this.props.mouse_event_topic !== undefined) ? this.props.mouse_event_topic : null



    var panGoalRatio = 0.5
    var tiltGoalRatio = 0.5
    if (status_msg != null){
      panGoalRatio = status_msg.pan_goal_ratio
      tiltGoalRatio = status_msg.tilt_goal_ratio
    }
   

    const svxDevicesList = Object.keys(svxDevices)
    var has_abs_pos = false
    var has_timed_pos = false
    var has_timed_speed_pos = false
    if (svxDevicesList.indexOf(namespace) !== -1){
      const svx_caps = svxDevices[namespace]
      has_abs_pos = svx_caps && (svx_caps.has_absolute_positioning === true)
      has_timed_pos = svx_caps && (svx_caps.has_timed_positioning === true)
      has_timed_speed_pos = svx_caps && (svx_caps.has_timed_speed_positioning === true)
    }

    const svxImageViewerElement = document.getElementById("svxImageViewer")
    const tiltSliderHeight = (svxImageViewerElement)? Math.floor(svxImageViewerElement.offsetHeight * 0.8) : 1
    const show_sv_controls = (tiltSliderHeight === 1) ? false : (has_abs_pos === true)


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
          <Column style={{flex: 0.05}}>

           <div hidden={show_sv_controls === false}>

            <SliderAdjustment
              title={"Tilt"}
              msgType={"std_msgs/Float32"}
              adjustment={tiltGoalRatio}
              topic={namespace + "/goto_tilt_ratio"}
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



              <div hidden={show_sv_controls === false}>

                  <SliderAdjustment
                    title={"Pan"}
                    msgType={"std_msgs/Float32"}
                    adjustment={panGoalRatio}
                    topic={namespace + "/goto_pan_ratio"}
                    scaled={0.01}
                    min={0}
                    max={100}
                    tooltip={"Pan as a percentage (0%=min, 100%=max)"}
                    unit={"%"}
                    noTextBox={true}
                    noLabel={true}
                  />
              </div>

              {(has_timed_pos === true) ?

                      <div style={{display: 'flex', alignItems: 'center', width: '100%'}}>

                        {(has_timed_speed_pos === true) &&
                          <div style={{flex: 1, paddingRight: '8px', display: 'flex', flexDirection: 'column', alignItems: 'stretch'}}>
                            <span style={{color: 'white', textAlign: 'center', fontSize: '11px', marginBottom: '2px'}}>
                              {"Jog Speed " + Math.round(jog_speed_ratio * 100) + "%"}
                            </span>
                            <input
                              type="range"
                              min="1"
                              max="100"
                              value={Math.round(jog_speed_ratio * 100)}
                              onChange={(e) => this.setState({ jog_speed_ratio: e.target.value / 100 })}
                              style={{width: '100%'}}
                            />
                          </div>
                        }

                        <div style={{flex: 1, display: 'flex', justifyContent: 'center'}}>
                          <ButtonMenu>

                            <Button
                              buttonDownAction={() => has_timed_speed_pos ? onSVXJogSpeedPan(namespace,  1, jog_speed_ratio) : onSVXJogPan(namespace,  1)}
                              buttonUpAction={() => onSVXPanStop(namespace)}>
                              {'\u25C0'}
                            </Button>
                            <Button
                              buttonDownAction={() => has_timed_speed_pos ? onSVXJogSpeedPan(namespace, -1, jog_speed_ratio) : onSVXJogPan(namespace, -1)}
                              buttonUpAction={() => onSVXPanStop(namespace)}>
                              {'\u25B6'}
                            </Button>
                            <Button
                              buttonDownAction={() => has_timed_speed_pos ? onSVXJogSpeedTilt(namespace, -1, jog_speed_ratio) : onSVXJogTilt(namespace, -1)}
                              buttonUpAction={() => onSVXTiltStop(namespace)}>
                              {'\u25B2'}
                            </Button>
                            <Button
                              buttonDownAction={() => has_timed_speed_pos ? onSVXJogSpeedTilt(namespace,  1, jog_speed_ratio) : onSVXJogTilt(namespace,  1)}
                              buttonUpAction={() => onSVXTiltStop(namespace)}>
                              {'\u25BC'}
                            </Button>

                            <Button onClick={() => onSVXStop(namespace)}>{"STOP"}</Button>

                          </ButtonMenu>
                        </div>

                      </div>

                    :

                      <ButtonMenu>

                          <Button onClick={() => onSVXStop(namespace)}>{"STOP"}</Button>

                        </ButtonMenu>

                    }

  </React.Fragment>




    )
  }


}

export default NepiDeviceSVXImageViewer
