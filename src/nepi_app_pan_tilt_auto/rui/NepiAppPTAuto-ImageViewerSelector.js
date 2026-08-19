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

import Section from "./Section"
//import EnableAdjustment from "./EnableAdjustment"
import Button, { ButtonMenu } from "./Button"
import { Column, Columns } from "./Columns"
import Styles from "./Styles"
import Select, { Option } from "./Select"

import ImageViewer from "./Nepi_IF_ImageViewer"
import {createMenuFirstLastName, createMenuFirstLastNames} from "./Utilities"


@inject("ros")
@observer
class NepiIFImageViewerSelector extends Component {
  constructor(props) {
    super(props)

    this.state = {


      hide_list: true,

      image_topics: [],
      image_topics_names: [],
      filter_list: [],
      id: '0',
      selected_image: 'None',
      selected_image_index: -1,
      selected_image_text: 'None',

      connected: true
    }
    this.renderNepiIFImageViewerSelector = this.renderNepiIFImageViewerSelector.bind(this)
    this.renderButtonControls = this.renderButtonControls.bind(this)
  
    this.getImageMenu = this.getImageMenu.bind(this)
    this.toggleViewableList = this.toggleViewableList.bind(this)
    this.onToggleListSelection = this.onToggleListSelection.bind(this)

  }



  toggleViewableList() {
    const show_list = ((this.state.hide_list === true) && (this.state.connected === true))
    if (show_list === false){
      this.setState({hide_list: !show_list,
        selected_image: 'None',
      })
    }
    else {
      this.setState({hide_list: !show_list
      })

    }
  }


  // Function for creating list menu options.
  getImageMenu() {
    // Update Class List
    const image_topics = (this.props.image_topics !== undefined) ? this.props.image_topics : this.props.ros.imageTopics
    const image_exclude_filters = (this.props.image_exclude_filters !== undefined) ? this.props.image_exclude_filters : []
    const image_include_filters = (this.props.image_include_filters !== undefined) ? this.props.image_include_filters : []
    const auto_select_image = (this.props.auto_select_image !== undefined) ? this.props.auto_select_image : true
    var images = image_topics  
    var items = []
    var push_item = true
    var image = ''
    if (images.length > 0){
      for (var i = 0; i < images.length; i++) {
        image = images[i]
        push_item = true
        for (var i2 = 0; i2 < image_exclude_filters.length; i2++) {
          if (image.indexOf(image_exclude_filters[i2]) !== -1 ){
            push_item = false
          }
        }
        for (i2 = 0; i2 < image_include_filters.length; i2++) {
          if (image.indexOf(image_include_filters[i2]) === -1 ){
            push_item = false
          }
        }

        if (push_item === true){
          items.push(image)
        }
      }
    }


    // Create Menu List
    var menu_items = []
    var item_names = createMenuFirstLastNames(items)
    if (items.length > 0){
      for (i = 0; i < items.length; i++) {
          menu_items.push(<Option value={items[i]}>{item_names[i]}</Option>)
        
      }
    }
    // if (menu_items.length == 0){
    //   menu_items.push(<Option value={'None'}>{'None'}</Option>)
    // }




    return menu_items

  }



  onToggleListSelection(event){

    const selected_image = event.target.value
    const text = event.target.text
    const image_topics = this.state.image_topics
    const index = image_topics.indexOf(selected_image)

    this.setState({
                    selected_image: selected_image,
                    selected_image_index: index,
                  selected_image_text: text})
    this.setState({hide_list: true})

    const {sendStringMsg} = this.props.ros
    const select_updated_topic = this.props.select_updated_topic ? this.props.select_updated_topic : null
    if ((select_updated_topic != null && selected_image !== undefined && selected_image != null)){
      sendStringMsg(select_updated_topic,selected_image)
    }

  }

  renderNepiIFImageViewerSelector() {
    const hide_list = ((this.state.hide_list === true) || (this.state.connected === false))
    const menu_options = this.getImageMenu()
    const selected_item = this.state.selected_image
    //const selected_name = this.state.selected_name
    const active_list = []


    const image_topics = this.state.image_topics
    //const image_topic = (this.props.image_topic !== undefined) ? this.props.image_topic : this.state.selected_image
    const show_selector = (this.props.show_selector !== undefined ? this.props.show_selector : menu_options.length > 0)


    // const show_controls = ((menu_options.length > 0 && selected_item === 'None') || (menu_options.length > 1 )) && (show_selector === true )
    return (
      <React.Fragment>

          {show_selector === true ?

                <Columns>
                  <Column>
                  
                  <div style={{ marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

                    <div onClick={this.toggleViewableList} style={{backgroundColor: Styles.vars.colors.grey0}}>
                      <Select style={{width: "10px"}}/>
                    </div>

                    <div hidden={hide_list}>
                        {menu_options.map((list) =>
                        <div onClick={this.onToggleListSelection}
                          style={{
                            textAlign: "center",
                            padding: `${Styles.vars.spacing.xs}`,
                            color: Styles.vars.colors.black,
                            backgroundColor: (list.props.value === selected_item) ?
                              Styles.vars.colors.green :
                              (active_list.includes(list.props.value)) ? Styles.vars.colors.blue : Styles.vars.colors.grey0,
                            cursor: "pointer",
                            }}>
                            <body list-topic ={list} style={{color: Styles.vars.colors.black}}>{list}</body>
                        </div>
                        )}
                    </div>

                </Column>
                </Columns>
              :
                null
        }

      </React.Fragment>

    )
  }



  stepItem(step){
    const image_topics = this.state.image_topics
    var index = this.state.selected_image_index
    index = index + step
    if (index < 0){
      index = image_topics.length - 1
    }
    else if ( index >= image_topics.length ){
      index = 0
    }




    const selected_image = image_topics[index]
    const {sendStringMsg} = this.props.ros
    const select_updated_topic = this.props.select_updated_topic ? this.props.select_updated_topic : null
    if ((select_updated_topic != null && selected_image !== undefined && selected_image != null)){
      sendStringMsg(select_updated_topic,selected_image)
    }
    else {
      this.setState({selected_image_index: index,
                  selected_image: selected_image,
                  selected_image_text: createMenuFirstLastName(image_topics[index])})
    }
    this.setState({hide_list: true})

  }

  renderButtonControls() {
    const image_topics = this.state.image_topics

    const show_selector_buttons = this.props.show_selector_buttons !== undefined ? this.props.show_selector_buttons : true
    const show_controls = (image_topics.length > 1) && (show_selector_buttons === true )
    return (
      <React.Fragment>

          {show_controls === true ?
                <ButtonMenu>

                      <Button 
                        buttonUpAction={() => this.stepItem(-1)}>
                        {'\u25C0'}
                        </Button>
                      <Button 
                        buttonUpAction={() => this.stepItem(1)}>
                        {'\u25B6'}
                      </Button>
                  
                </ButtonMenu>
                :
                null

          }
      
      </React.Fragment>
    )
  }
  

  renderImageViewer() {

    const image_topic = (this.props.image_topic !== undefined) ? this.props.image_topic : this.state.selected_image
    const title = createMenuFirstLastName(image_topic)
    
    const image_index = (this.props.image_index !== undefined) ? this.props.image_index : 0
    const select_updated_topic = (this.props.select_updated_topic !== undefined) ? this.props.select_updated_topic : null
    const allow_pan_zoom = (this.props.allow_pan_zoom !== undefined) ? this.props.allow_pan_zoom : true
    const mouse_event_topic = (this.props.mouse_event_topic !== undefined) ? this.props.mouse_event_topic : null
    const mouse_click_topic = (this.props.mouse_click_topic !== undefined) ? this.props.mouse_click_topic : null
    const mouse_drag_topic = (this.props.mouse_drag_topic !== undefined) ? this.props.mouse_drag_topic : null
    const mouse_window_topic = (this.props.mouse_window_topic !== undefined) ? this.props.mouse_window_topic : null

    const streamingImageQuality = (this.props.streamingImageQuality !== undefined) ? 
                (this.props.streamingImageQuality != null) ? this.props.streamingImageQuality : 95
                : 95

    const streamingImageRate = (this.props.streamingImageRate !== undefined) ? 
            (this.props.streamingImageRate != null) ? this.props.streamingImageRate : 20
            : 20
    const show_image_controls = (this.props.show_image_controls !== undefined)? this.props.show_image_controls : true
    const show_info_controls = (this.props.show_info_controls !== undefined) ? this.props.show_info_controls : show_image_controls === true
    const show_config_controls = (this.props.show_config_controls !== undefined) ? this.props.show_config_controls : show_image_controls === true
    const show_navpose_controls = (this.props.show_navpose_controls !== undefined) ? this.props.show_navpose_controls : show_image_controls === true
    const show_render_controls = (this.props.show_render_controls !== undefined) ? this.props.show_render_controls : show_image_controls === true
    const show_overlay_controls = (this.props.show_overlay_controls !== undefined) ? this.props.show_overlay_controls : show_image_controls === true

    const show_save_controls = (this.props.show_save_controls !== undefined) ? this.props.show_save_controls : true
    const show_all_config_options = (this.props.show_all_config_options !== undefined) ? this.props.show_all_config_options : true
    const show_reset_button = (this.props.show_reset_button !== undefined) ? this.props.show_reset_button : true
    const show_browser_save_button = (this.props.show_browser_save_button !== undefined) ? this.props.show_browser_save_button : true
    const save_data_topic = this.props.save_data_topic

    return (

      <ImageViewer
      id="imageViewer"
      image_topic={image_topic}
      title={title}
      image_index={image_index}
      mouse_event_topic={mouse_event_topic}
      mouse_click_topic={mouse_click_topic}
      mouse_drag_topic={mouse_drag_topic}
      mouse_window_topic={mouse_window_topic}
      select_updated_topic={select_updated_topic}
      show_image_controls={show_image_controls}
      show_info_controls={show_info_controls}
      show_config_controls={show_config_controls}
      show_navpose_controls={show_navpose_controls}
      show_render_controls={show_render_controls}
      show_overlay_controls={show_overlay_controls}
      show_save_controls={show_save_controls}
      show_all_config_options={show_all_config_options}
      show_reset_button={show_reset_button}
      show_browser_save_button={show_browser_save_button}
      allow_pan_zoom={allow_pan_zoom}
      save_data_topic={save_data_topic}
      make_section={false}
      streamingImageQuality={streamingImageQuality}
      streamingImageRate={streamingImageRate}

    />


    )
  }



  render() {
    const make_section = (this.props.make_section !== undefined)? this.props.make_section : true
    const hide_image = this.state.hide_list === false


    if (make_section === false){
      return (
          <React.Fragment>

        <div style={{ display: 'flex' }}>
              <div style={{ width: '30%' }}>
                {this.renderNepiIFImageViewerSelector()}
              </div>

                <div style={{ width: '50%' }}>
                  {}
                </div>
                
                <div style={{ width: '20%' }} hidden={hide_image}>
                  {this.renderButtonControls()}
                </div>
        </div>


        <div  hidden={hide_image}>
          {this.renderImageViewer()}
        </div>
          </React.Fragment>
      )
    }
    else {
      return (

      <Section>

          <div style={{ display: 'flex' }}>
              <div style={{ width: '30%' }}>
                {this.renderNepiIFImageViewerSelector()}
              </div>

                <div style={{ width: '50%' }}>
                  {}
                </div>

                <div style={{ width: '20%' }}>
                  {this.renderButtonControls()}
                </div>
        </div>
        {this.renderImageViewer()}

      </Section>
      )

    }
  }

}

export default NepiIFImageViewerSelector
