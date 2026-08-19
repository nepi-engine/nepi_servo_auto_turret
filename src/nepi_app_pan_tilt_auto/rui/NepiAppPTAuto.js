/*
 * Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
 *
 * This file is part of nepi-engine
 * (see https://github.com/nepi-engine).
 *
 * License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
 */
import React, { Component } from "react"
import { observer, inject } from "mobx-react"
//import Toggle from "react-toggle"

import Section from "./Section"
import { Columns, Column } from "./Columns"
import Select, { Option } from "./Select"
import Label from "./Label"
//import Input from "./Input"
import Styles from "./Styles"
//import Button, { ButtonMenu } from "./Button"
//import {setElementStyleModified, clearElementStyleModified, onUpdateSetStateValue} from "./Utilities"
//import {createShortValuesFromNamespaces} from "./Utilities"

//import {onChangeSwitchStateValue } from "./Utilities"


import NepiPTAutoImageViewer from "./NepiAppPTAuto-ImageViewer"
import NepiAppPTAutoControls from "./NepiAppPTAuto-Controls"

//import NavPoseViewer from "./Nepi_IF_NavPoseViewer"
import NepiIFSaveData from "./Nepi_IF_SaveData"
import NepiIFConfig from "./Nepi_IF_Config"


@inject("ros")
@observer

// Component that contains the  controls
class NepiAppPTAuto extends Component {
  constructor(props) {
    super(props)

    this.state = {
      appName: 'app_pan_tilt_auto',
      appNamespace: null,
      status_msg: null,

      available_pan_tilts: [],
      selected_pan_tilt: null,
      pt_connected: false,
      pt_connected_topic: null,

      selected_image_topics: ['None','None','None','None'],
      display_image_topics: ['None','None','None','None'],
      single_image_index: 'None',
      num_windows: 1,

      statusListener: null,
      needs_update: false,


    }

    this.renderControls = this.renderControls.bind(this)
    this.renderSaveData = this.renderSaveData.bind(this)
    this.renderConfig = this.renderConfig.bind(this)
    
    this.createPtMenuOptions = this.createPtMenuOptions.bind(this)
    this.onClickToggleShowSettings = this.onClickToggleShowSettings.bind(this)
    this.onPtDeviceSelected = this.onPtDeviceSelected.bind(this)

    this.getAllSaveNamespace = this.getAllSaveNamespace.bind(this)
    this.getAppNamespace = this.getAppNamespace.bind(this)
    this.updateStatusListener = this.updateStatusListener.bind(this)
    this.statusListener = this.statusListener.bind(this)
  }


  getAllSaveNamespace(){
    const { namespacePrefix, deviceId} = this.props.ros
    var allNamespace = null
    if (namespacePrefix !== null && deviceId !== null){
      allNamespace = "/" + namespacePrefix + "/" + deviceId + '/save_data'
    }
    return allNamespace
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
    this.setState({
      status_msg: message,
      available_pan_tilts: message.available_pan_tilts,
      selected_pan_tilt: message.selected_pan_tilt,
      pt_connected: message.pt_connected,
      pt_connected_topic: message.pt_connected_topic,
      selected_image_topics: message.selected_image_topics,
      display_image_topics: message.display_image_topics,
      single_image_index: message.single_image_index,
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



  // Function for creating topic options for Select input
  createPtMenuOptions() {
    const {sendStringMsg} = this.props.ros
    const namespace = this.getAppNamespace()
    const topics = this.state.available_pan_tilts
    const sel_topic = this.state.selected_pan_tilt
    var items = []
    var i
    //var unique_names = createShortUniqueValues(topics)
    var device_name = ""

    if (topics.length > 0){
      for (i = 0; i < topics.length; i++) {
        device_name = topics[i].split('/ptx')[0].split('/').pop()
        items.push(<Option value={topics[i]}>{device_name}</Option>)
      }
    }

    if (topics.length === 0){
      items.push(<Option value={"None"}>{"None Availble"}</Option>)
    }
    if (sel_topic === 'None' && topics.length > 0){
          this.setState({selected_pan_tilt: topics[0]})
          const selectNamespace = namespace + "/select_pt_device"
          sendStringMsg(selectNamespace,topics[0])
    }
    return items
  }




  onClickToggleShowSettings(){
    const currentVal = this.state.showSettings 
    this.setState({showSettings: !currentVal})
    this.render()
  }

  onPtDeviceSelected(event) {
    const {sendStringMsg} = this.props.ros
    const namespace = this.getAppNamespace()
    const item = event.target.value
    //var item_ind = this.ordered_items_list.index(item)
    //if (item_ind != -1){
    this.setState({selected_pan_tilt: item})
    const selectNamespace = namespace + "/select_pt_device"
    sendStringMsg(selectNamespace,item)
   // }
  }


  renderConfig(){
    const namespace = this.getAppNamespace()
    const ptConnected = this.state.pt_connected
    const pt_connected_topic = this.state.pt_connected_topic
    var add_namspaces = []
    if (ptConnected === true) {
      add_namspaces = [pt_connected_topic]
    }

    return (
  
    <React.Fragment>

           <NepiIFConfig
                namespace={namespace}
                title={"Nepi_IF_Config"}
                show_save_all={false}
                add_namspaces={add_namspaces}
                restricted={false}
                make_section={false}
            />

    </React.Fragment>

    )
}



  renderControls() {

    const appNamespace = this.getAppNamespace()
    const selected_pan_tilt = this.state.selected_pan_tilt
    //const ptConnected = this.state.pt_connected
    const ptMenuItems = this.createPtMenuOptions()
    //const show_pt_selector = (ptMenuItems.length > 1) ? true : (ptConnected === false)
    return (




      
      <React.Fragment>


          <Section title={"PAN TILT AUTOMATION"}>





            <Label title={"Pan Tilt Device"}>
                <Select
                  onChange={this.onPtDeviceSelected}
                  value={selected_pan_tilt}
                >
                  {ptMenuItems}
                </Select>
          </Label>
  
            {/* {(ptConnected === false) ?
                  <Label title={"Connecting"}>
                    </Label>
            : null } */}


          {/* { (ptConnected === true) ?
            <div style={{ borderTop: "1px solid #000000", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>
          : null } */}
          <div style={{ borderTop: "1px solid #000000", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

          {/* { (ptConnected === true) ?
            <NepiAppPTAutoControls
                namespace={appNamespace}
                make_section={false}
            />
          : null } */}
          <NepiAppPTAutoControls
              namespace={appNamespace}
              make_section={false}
          />

            { this.renderConfig() }





          </Section>



          </React.Fragment>
        )
  }



    renderSaveData(){
      const allSaveNamespace = this.getAllSaveNamespace()
      const saveNamespace = allSaveNamespace
      const show_save_controls = (this.props.show_save_controls !== undefined) ? this.props.show_save_controls : true

      if (show_save_controls === false){
          return (
            <Columns>
            <Column>

            </Column>
            </Columns>

          )

      }
      else {
          return (
        
              <React.Fragment>

                         
                          
                          <NepiIFSaveData
                            saveNamespace={saveNamespace}
                            make_section={true}
                            show_all_options={true}
                            allow_enable={false}
                            show_topic_selector={true}
                          />

              </React.Fragment>

          )
        }
  }



    
  render() {
    const ptConnected = this.state.pt_connected
    const num_windows = this.state.num_windows
    const selected_image_topics = this.state.selected_image_topics
    const display_image_topics = this.state.display_image_topics
    const single_image_index = this.state.single_image_index
    const clicking_enabled = (this.state.status_msg != null) ? (this.state.status_msg.click_pan_enabled === true || this.state.status_msg.click_tilt_enabled === true) : false
    const namespace = this.getAppNamespace()
    const mouse_event_topic = (clicking_enabled === true) ? namespace + '/set_click_position' : null
    const full_screen_enabled = (this.state.status_msg != null) ? this.state.status_msg.full_screen_enabled : false

  if (full_screen_enabled === true){
    return (
        <React.Fragment>

              <div id="ptAutoImageViewer">
                      <NepiPTAutoImageViewer
                        id="ptAutoImageViewer"
                        show_image_options={false}
                        namespace={namespace}
                        mouse_event_topic={mouse_event_topic}
                        num_windows={num_windows}
                        image_topics={display_image_topics}
                        single_image_index={single_image_index}

                      />
                    </div>


        </React.Fragment>
      )
  }
  else {
      return (
        <React.Fragment>



        <div style={{ display: 'flex' }}>

              <div style={{ width: '75%' }}>

              <div id="ptAutoImageViewer">
                      <NepiPTAutoImageViewer
                        id="ptAutoImageViewer"
                        show_image_options={false}
                        namespace={namespace}
                        mouse_event_topic={mouse_event_topic}
                        num_windows={num_windows}
                        image_topics={display_image_topics}
                        single_image_index={single_image_index}

                      />
                    </div>



                  { this.renderSaveData() }

              </div>

              <div style={{ width: '2%' }} centered={"true"} >
                    {}
              </div>

              <div style={{ width: '23%' }}>


              { this.renderControls()}


              </div>
        </div>

        </React.Fragment>
      )
    }
  }
}

export default NepiAppPTAuto
