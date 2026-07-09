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

//import moment from "moment"
import { observer, inject } from "mobx-react"

//import { TransformWrapper, TransformComponent } from "react-zoom-pan-pinch";

import Button, { ButtonMenu } from "./Button"

import { Column, Columns } from "./Columns"


import { SliderAdjustment } from "./AdjustmentWidgets"

import NepiIFImageViewersSelector from "./Nepi_IF_ImageViewersSelector"
import {createMenuFirstLastNames} from "./Utilities"



@inject("ros")
@observer
class NepiAppSVAutoImageViewer extends Component {
  constructor(props) {
    super(props)

    this.state = {

      appNamespace: null,
      status_msg: null, 

      available_servos: [],
      selected_servo: null,
      connected: false,
      connected_topic: null,

      image_topics: ['None','None','None','None'],
      num_windows: 1,


      statusSvListener: null,
      sv_status_msg: null,


 
      statusListener: null,
      needs_update: false
    }


    
    this.renderImageViewers = this.renderImageViewers.bind(this)

    this.updateStatusSvListener = this.updateStatusSvListener.bind(this)
    this.statusSvListener = this.statusSvListener.bind(this)

    this.getAppNamespace = this.getAppNamespace.bind(this)
    this.updateStatusListener = this.updateStatusListener.bind(this)
    this.statusListener = this.statusListener.bind(this)
  }


  getAppNamespace(){
    const { namespacePrefix, deviceId} = this.props.ros
    var namespace = null
    if (namespacePrefix != null && deviceId != null){
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
    if ((this.state.selected_servo !== message.selected_servo) && (message.selected_servo !== '' && message.selected_servo !== 'None')) {
      this.updateStatusSvListener(message.selected_servo)
    }
    this.setState({
      status_msg: message,
      available_servos: message.available_servos,
      selected_servo: message.selected_servo,
      connected: message.connected,
      image_topics: message.image_topics,
      num_windows: message.num_windows
    })
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

  // Callback for handling ROS Status3DX messages
  statusSvListener(message) {
    this.setState({
      sv_status_msg: message
    })
  }
  
  // Function for configuring and subscribing to Status
  updateStatusSvListener(namespace) {
    if (this.state.statusSvListener != null) {
      this.state.statusSvListener.unsubscribe()
       this.setState({ sv_status_msg: null, statusSvListener: null})
    }
    if (namespace != null && namespace !== 'None'){
        var statusSvListener = this.props.ros.setupSVXStatusListener(
              namespace,
              this.statusSvListener
            )
      this.setState({ statusSvListener: statusSvListener})
    }

}
  
// Lifecycle method called when compnent updates.
// Used to track changes in the topic
componentDidUpdate(prevProps, prevState, snapshot) {
  const namespace = this.getAppNamespace()
   if ((namespace != null && namespace !== this.state.appNamespace) || this.state.needs_update === true){
      this.updateStatusListener(namespace)
  }
}

  componentDidMount() {
    this.setState({needs_update: true
    })
    }



  // Lifecycle method called just before the component umounts.
  // Used to unsubscribe to Status3DX message
  componentWillUnmount() {
    if (this.state.statusListener) {
      this.state.statusListener.unsubscribe()
      this.setState({statusListener : null})
    }
  }








  renderImageViewers() {
     if (this.state.needs_update === true){
      this.setState({needs_update: false})
    }
    const num_windows = (this.props.num_windows !== undefined) ? this.props.num_windows : this.state.num_windows
    const image_topics = (this.props.image_topics !== undefined) ? this.props.image_topics : ['None','None','None','None']
    const namespace = (this.props.namespace !== null) ? this.props.namespace : 'None'
    //Unused const baseNamespace = "/" + namespacePrefix + "/" + deviceId 
    const topics_text = createMenuFirstLastNames(image_topics)
    const image_filters = ['/all/']
    const num_windows_updated_topic = namespace + '/set_num_windows'
    const select_updated_topics = [
        namespace + '/set_topic_1',
        namespace + '/set_topic_2',
        namespace + '/set_topic_3',
        namespace + '/set_topic_4'
    ]

    const mouse_click_topics = [
        namespace + '/set_mouse_click',
        namespace + '/set_mouse_click',
        namespace + '/set_mouse_click',
        namespace + '/set_mouse_click'
    ]


    const set_image_priority_callback = namespace + '/set_image_priority'
    const image_priority_options = (this.state.status_msg != null) ? this.state.status_msg.image_priority_options : []

      return (
     

      <React.Fragment>


                          <div id="imageviewers">
                            <NepiIFImageViewersSelector
                              id="imageviewers"
                              image_topics={image_topics}
                              titles={topics_text}
                              show_save_controls={false}
                              show_browser_save_button={true}
                              num_windows={num_windows}
                              num_windows_updated_topic={num_windows_updated_topic}
                              select_updated_topics={select_updated_topics}
                              mouse_click_topics={mouse_click_topics}
                              custom_selection_options={image_priority_options}
                              custom_selection_callback={set_image_priority_callback}
                              image_filters={image_filters}
                              auto_select_image={false}
                              make_section={true}
                            />
                          </div>        
 
      </React.Fragment>

      )
  }



  render() {
    const { svxDevices, onSVXJogPan, onSVXJogTilt, onSVXStop, onSVXPanStop, onSVXTiltStop } = this.props.ros
    const sv_status_msg = this.state.sv_status_msg

    var panGoalRatio = 0.5
    var tiltGoalRatio = 0.5
    if (sv_status_msg != null){
      panGoalRatio = sv_status_msg.pan_goal_ratio
      tiltGoalRatio = sv_status_msg.tilt_goal_ratio
    }
   

    const svxDevicesList = Object.keys(svxDevices)
    var has_abs_pos = false
    var has_timed_pos = false

    const svNamespace = this.state.selected_servo
    if (svxDevicesList.indexOf(svNamespace) !== -1){
      const svx_caps = svxDevices[svNamespace]
      has_abs_pos = svx_caps && (svx_caps.has_absolute_positioning === true)
      has_timed_pos = svx_caps && (svx_caps.has_timed_positioning === true)
    }

    const imageviewersElement = document.getElementById("imageviewers")
    const tiltSliderHeight = (imageviewersElement)? Math.floor(imageviewersElement.offsetHeight * 1.0) : 1
    const show_sv_controls = (tiltSliderHeight === 1) ? false : (has_abs_pos === true)


    var pan_slider_disabled = false
    var pan_slider_topic = svNamespace + "/goto_pan_ratio"
    var tilt_slider_disabled = false
    var tilt_slider_topic = svNamespace + "/goto_tilt_ratio"
    const namespace = (this.props.namespace !== null) ? this.props.namespace : 'None'
    const status_msg = this.state.status_msg
    if (status_msg != null){
      pan_slider_disabled = (status_msg.pan_scanning === true || status_msg.pan_tracking === true)
      const pan_stabing = status_msg.pan_stabing
      if (pan_stabing === true){
        pan_slider_topic = namespace  + "/set_stab_pan_pos_ratio"
      }
      tilt_slider_disabled = (status_msg.tilt_scanning === true || status_msg.tilt_tracking === true)
      const tilt_stabing = status_msg.tilt_stabing
      if (tilt_stabing === true){
        tilt_slider_topic = namespace  + "/set_stab_tilt_pos_ratio"
      }
    }
  
    return (




        <Columns>
          <Column equalWidth = {false} >

  

          {this.renderImageViewers()}


              <div hidden={show_sv_controls === false}>
        
                  <SliderAdjustment
                    title={"Pan"}
                    msgType={"std_msgs/Float32"}
                    adjustment={panGoalRatio}
                    disabled={pan_slider_disabled === true}
                    topic={pan_slider_topic}
                    scaled={0.01}
                    min={0}
                    max={100}
                    tooltip={"Pan as a percentage (0%=min, 100%=max)"}
                    unit={"%"}
                    noTextBox={true}
                    noLabel={true}
                  />
        


              {(has_timed_pos === true) ?

                      <ButtonMenu>

                          <Button 
                            buttonDownAction={() => onSVXJogPan(svNamespace,  1)}
                            buttonUpAction={() => onSVXPanStop(svNamespace)}>
                            {'\u25C0'}
                            </Button>
                          <Button 
                            buttonDownAction={() => onSVXJogPan(svNamespace, - 1)}
                            buttonUpAction={() => onSVXPanStop(svNamespace)}>
                            {'\u25B6'}
                          </Button>
                          <Button 
                            buttonDownAction={() => onSVXJogTilt(svNamespace, -1)}
                            buttonUpAction={() => onSVXTiltStop(svNamespace)}>
                            {'\u25B2'}
                          </Button>
                          <Button 
                            buttonDownAction={() => onSVXJogTilt(svNamespace, 1)}
                            buttonUpAction={() => onSVXTiltStop(svNamespace)}>
                            {'\u25BC'}
                          </Button>

                          <Button onClick={() => onSVXStop(svNamespace)}>{"STOP"}</Button>

                        </ButtonMenu>

                    : 

                      <ButtonMenu>

                          <Button onClick={() => onSVXStop(svNamespace)}>{"STOP"}</Button>

                        </ButtonMenu>

                    }

               

             </div>


          </Column>
          <Column style={{flex: 0.05}}>

           <div hidden={show_sv_controls === false}>

            <SliderAdjustment
              title={"Tilt"}
              msgType={"std_msgs/Float32"}
              adjustment={tiltGoalRatio}
              disabled={tilt_slider_disabled === true}
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



    )
  }


}

export default NepiAppSVAutoImageViewer
