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

import ImageViewersSelector from "./NepiAppPTAuto-ImageViewersSelector"
import {createMenuFirstLastNames} from "./Utilities"



@inject("ros")
@observer
class NepiAppPTAutoImageViewer extends Component {
  constructor(props) {
    super(props)

    this.state = {

      appNamespace: null,
      status_msg: null, 

      available_pan_tilts: [],
      selected_pan_tilt: null,
      connected: false,
      connected_topic: null,

      image_topics: ['None','None','None','None'],
      num_windows: 1,


      statusPtListener: null,
      pt_status_msg: null,


 
      statusListener: null,
      needs_update: false
    }


    
    this.renderImageViewers = this.renderImageViewers.bind(this)

    this.updateStatusPtListener = this.updateStatusPtListener.bind(this)
    this.statusPtListener = this.statusPtListener.bind(this)

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
    if ((this.state.selected_pan_tilt !== message.selected_pan_tilt) && (message.selected_pan_tilt !== '' && message.selected_pan_tilt !== 'None')) {
      this.updateStatusPtListener(message.selected_pan_tilt)
    }
    this.setState({
      status_msg: message,
      available_pan_tilts: message.available_pan_tilts,
      selected_pan_tilt: message.selected_pan_tilt,
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
              "nepi_app_pan_tilt_auto/PanTiltAutoAppStatus",
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
  statusPtListener(message) {
    this.setState({
      pt_status_msg: message
    })
  }
  
  // Function for configuring and subscribing to Status
  updateStatusPtListener(namespace) {
    if (this.state.statusPtListener != null) {
      this.state.statusPtListener.unsubscribe()
       this.setState({ pt_status_msg: null, statusPtListener: null})
    }
    if (namespace != null && namespace !== 'None'){
        var statusPtListener = this.props.ros.setupPTXStatusListener(
              namespace,
              this.statusPtListener
            )
      this.setState({ statusPtListener: statusPtListener})
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
    const single_image_index = (this.props.single_image_index !== undefined) ? this.props.single_image_index : 0
    const namespace = (this.props.namespace !== null) ? this.props.namespace : 'None'
    //Unused const baseNamespace = "/" + namespacePrefix + "/" + deviceId 
    const topics_text = createMenuFirstLastNames(image_topics)
    const image_exclude_filters = ['/all/','detections_image','targets_image','track_image']
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

    const full_screen_enabled = (this.state.status_msg != null) ? this.state.status_msg.full_screen_enabled : false
    const full_screen_update_topic = namespace + '/set_full_screen_enable'

    const dual_mode_supported = (this.state.status_msg != null) ? this.state.status_msg.has_dual_mode : false
    const dual_mode_enabled = (this.state.status_msg != null) ? this.state.status_msg.dual_mode_enabled : false
    const dual_mode_update_topic = namespace + '/set_image_dual_enable'

    const night_mode_supported = (this.state.status_msg != null) ? this.state.status_msg.has_night_mode : false
    const night_mode_enabled = (this.state.status_msg != null) ? this.state.status_msg.night_mode_enabled : false
    const night_mode_update_topic = namespace + '/set_image_night_enable'

    const zoom_mode_supported = (this.state.status_msg != null) ? this.state.status_msg.has_zoom_mode : false
    const zoom_mode_enabled = (this.state.status_msg != null) ? this.state.status_msg.zoom_mode_enabled : false
    const zoom_mode_update_topic = namespace + '/set_image_zoom_enable'

    const detect_mode_supported  = (this.state.status_msg != null) ? this.state.status_msg.has_detect_mode : false
    const detect_mode_enabled = (this.state.status_msg != null) ? this.state.status_msg.detect_mode_enabled : false
    const detect_mode_update_topic = namespace + '/set_image_detect_enable'

    const image_stab_supported = (this.state.status_msg != null) ? this.state.status_msg.has_image_stab : false
    const image_stab_enabled = (this.state.status_msg != null) ? this.state.status_msg.image_stab_enabled : false
    const image_stab_update_topic = namespace + '/set_image_stab_enable'

    const stream_quality = (this.state.status_msg != null) ? this.state.status_msg.stream_quality : 1
    const stream_quality_update_topic = namespace + '/set_image_stream_quality'

    const stream_rate = (this.state.status_msg != null) ? this.state.status_msg.stream_rate : 1
    const stream_rate_update_topic = namespace + '/set_image_stream_rate'

      return (
     

      <React.Fragment>


                          <div id="imageviewers">
                            <ImageViewersSelector
                              id="imageviewers"
                              full_screen_enabled={full_screen_enabled}
                              full_screen_update_topic={full_screen_update_topic}
                              dual_mode_supported={dual_mode_supported}
                              dual_mode_enabled={dual_mode_enabled}
                              dual_mode_update_topic={dual_mode_update_topic}
                              night_mode_supported={dual_mode_supported}
                              night_mode_enabled={night_mode_enabled}
                              night_mode_update_topic={night_mode_update_topic}
                              night_mode_supported={night_mode_supported}
                              zoom_mode_supported={zoom_mode_supported}
                              zoom_mode_enabled={zoom_mode_enabled}
                              zoom_mode_update_topic={zoom_mode_update_topic}
                              detect_mode_supported={detect_mode_supported}
                              detect_mode_enabled={detect_mode_enabled}
                              detect_mode_update_topic={detect_mode_update_topic}
                              image_stab_supported={image_stab_supported}
                              image_stab_enabled={image_stab_enabled}
                              image_stab_update_topic={image_stab_update_topic}
                              stream_quality_ratio={stream_quality}
                              stream_quality_update_topic={stream_quality_update_topic}
                              stream_rate_ratio={stream_rate}
                              stream_rate_update_topic={stream_rate_update_topic}
                              image_topics={image_topics}
                              single_image_index={single_image_index}
                              titles={topics_text}
                              num_windows={num_windows}
                              num_windows_updated_topic={num_windows_updated_topic}
                              select_updated_topics={select_updated_topics}
                              mouse_click_topics={mouse_click_topics}
                              custom_selection_options={image_priority_options}
                              custom_selection_callback={set_image_priority_callback}
                              image_exclude_filters={image_exclude_filters}
                              auto_select_image={false}
                              make_section={true}
                              show_save_controls={false}
                            />
                          </div>        
 
      </React.Fragment>

      )
  }



  render() {
    const { ptxDevices, onPTXJogPan, onPTXJogTilt, onPTXStop, onPTXPanStop, onPTXTiltStop } = this.props.ros
    const pt_status_msg = this.state.pt_status_msg
    const status_msg = this.state.status_msg



    const ptxDevicesList = Object.keys(ptxDevices)
    var has_abs_pos = false
    var has_timed_pos = false

    const ptNamespace = this.state.selected_pan_tilt
    if (ptxDevicesList.indexOf(ptNamespace) !== -1){
      const ptx_caps = ptxDevices[ptNamespace]
      has_abs_pos = ptx_caps && (ptx_caps.has_absolute_positioning === true)
      has_timed_pos = ptx_caps && (ptx_caps.has_timed_positioning === true)
    }

    const imageviewersElement = document.getElementById("imageviewers")
    const tiltSliderHeight = (imageviewersElement)? Math.floor(imageviewersElement.offsetHeight * 1.0) : 1
    const show_pt_controls = (tiltSliderHeight === 1) ? false : (has_abs_pos === true)

        var panGoalRatio = 0.5
    var tiltGoalRatio = 0.5
    if (status_msg != null){
      panGoalRatio = status_msg.auto_pan_ratio_set
      tiltGoalRatio = status_msg.auto_tilt_ratio_set
    }
   
    var panSliderDisabled = true
    var tiltSliderDisabled = true
    if (status_msg != null){
      panSliderDisabled = status_msg.auto_pan_ratio_disabled
      tiltSliderDisabled = status_msg.auto_tilt_ratio_disabled
    }

    var pan_slider_topic = ptNamespace + "/goto_pan_ratio"
    var tilt_slider_topic = ptNamespace + "/goto_tilt_ratio"
    const namespace = (this.props.namespace !== null) ? this.props.namespace : 'None'
    if (status_msg != null){
      pan_slider_topic = namespace  + "/set_stab_pan_pos_ratio"
      tilt_slider_topic = namespace  + "/set_stab_tilt_pos_ratio"

    }
  
    return (




        <Columns>
          <Column equalWidth = {false} >

  

          {this.renderImageViewers()}


              <div hidden={show_pt_controls === false}>
        
                  <SliderAdjustment
                    title={"Pan"}
                    msgType={"std_msgs/Float32"}
                    adjustment={panGoalRatio}
                    disabled={panSliderDisabled === true}
                    topic={pan_slider_topic}
                    scaled={0.01}
                    min={0}
                    max={100}
                    tooltip={"Pan as a percentage (0%=min, 100%=max)"}
                    unit={"%"}
                    noTextBox={true}
                    noLabel={true}
                  />
        


              {/* {(has_timed_pos === true) ?

                      <ButtonMenu>

                          <Button 
                            buttonDownAction={() => onPTXJogPan(ptNamespace,  1)}
                            buttonUpAction={() => onPTXPanStop(ptNamespace)}>
                            {'\u25C0'}
                            </Button>
                          <Button 
                            buttonDownAction={() => onPTXJogPan(ptNamespace, - 1)}
                            buttonUpAction={() => onPTXPanStop(ptNamespace)}>
                            {'\u25B6'}
                          </Button>
                          <Button 
                            buttonDownAction={() => onPTXJogTilt(ptNamespace, -1)}
                            buttonUpAction={() => onPTXTiltStop(ptNamespace)}>
                            {'\u25B2'}
                          </Button>
                          <Button 
                            buttonDownAction={() => onPTXJogTilt(ptNamespace, 1)}
                            buttonUpAction={() => onPTXTiltStop(ptNamespace)}>
                            {'\u25BC'}
                          </Button>

                          <Button onClick={() => onPTXStop(ptNamespace)}>{"STOP"}</Button>

                        </ButtonMenu>

                    : 

                      <ButtonMenu>

                          <Button onClick={() => onPTXStop(ptNamespace)}>{"STOP"}</Button>

                        </ButtonMenu>

                    } */}

               

             </div>


          </Column>
          <Column style={{flex: 0.05}}>

           <div hidden={show_pt_controls === false}>

            <SliderAdjustment
              title={"Tilt"}
              msgType={"std_msgs/Float32"}
              adjustment={tiltGoalRatio}
              disabled={tiltSliderDisabled === true}
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

export default NepiAppPTAutoImageViewer
