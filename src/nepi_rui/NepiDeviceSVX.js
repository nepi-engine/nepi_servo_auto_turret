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


import Section from "./Section"
import { Columns, Column } from "./Columns"
import Select, { Option } from "./Select"
import Label from "./Label"

import NepiIFSettings from "./Nepi_IF_Settings"
import NepiIFAdmin from "./Nepi_IF_Admin"


import NepiDeviceSVXControls from "./NepiDeviceSVX-Controls"
import NepiDeviceSVXImageViewer from "./NepiDeviceSVX-ImageViewer"

@inject("ros")
@observer

// Component that contains the SVX controls
class NepiDeviceSVX extends Component {
  constructor(props) {
    super(props)

    this.state = {
      namespace: 'None',
      node_name: 'None',

    }

    this.renderImageViewer = this.renderImageViewer.bind(this)

    this.setDeviceSelection = this.setDeviceSelection.bind(this)
    this.clearDeviceSelection = this.clearDeviceSelection.bind(this)
    this.createDeviceOptions = this.createDeviceOptions.bind(this)
    this.onDeviceSelected = this.onDeviceSelected.bind(this)

    this.renderDeviceSelection = this.renderDeviceSelection.bind(this)

   }


  setDeviceSelection(namespace) {
      this.setState({
        namespace: namespace,
      })
  }

  clearDeviceSelection() {
    this.setState({
      namespace: 'None',
    })
  }

  // Function for creating topic options for Select input
  createDeviceOptions() {
    const { svxDevices} = this.props.ros
    const topics = Object.keys(svxDevices)
    const namespace = this.state.namespace
    var items = []
    items.push(<Option value={'None'}>{'None'}</Option>)
    var device_name = ""
    for (var i = 0; i < topics.length; i++) {
      device_name = topics[i].split('/svx')[0].split('/').pop()
      items.push(<Option value={topics[i]}>{device_name}</Option>)
    }
    // Check that our current selection hasn't disappeard as an available option
    if ((namespace !== null) && (namespace !== 'None') && (topics.includes(namespace) === false)) {
      this.clearDeviceSelection()
    }
    if (namespace !== 'None' && (topics.indexOf(namespace) === -1)){
      this.setState({namespace: 'None'})
    }
    return items
  }

  // Handler for SVX Sensor topic selection
  onDeviceSelected(event) {
    const value = event.target.value
      this.setDeviceSelection(value)
  }



  renderDeviceSelection() {
    const namespace = this.state.namespace ? this.state.namespace : "None"

      return(


          <Columns>
          <Column>
          
            <Label title={"Device"}>
              <Select
                onChange={this.onDeviceSelected}
                value={namespace}
              >
                {this.createDeviceOptions()}
              </Select>
            </Label>

          </Column>
          <Column>

          </Column>
        </Columns>
          


      )
  }



  renderImageViewer() {
    const namespace = (this.state.namespace !== null) ? this.state.namespace : "None"
    return (
      <React.Fragment>

                <div id="svxImageViewer">
                  <NepiDeviceSVXImageViewer
                    id="svxImageViewer"
                    namespace={namespace}
                  />
                </div>


      </React.Fragment>
    )
  }


 render() {
    const device_selected = (this.state.namespace !== null && this.state.namespace !== 'None')
    const namespace = (this.state.namespace !== null) ? this.state.namespace : 'None'
    const capabilities = this.props.ros.svxDevices[namespace]
    const node_name = capabilities ? capabilities.device_node_name : 'None'
    
        return (

          <Columns>
          <Column>


        
          <div style={{ display: 'flex' }}>

              <div style={{ width: "73%" }}>


              {(device_selected === true) ?
              this.renderImageViewer()
              : null}

              </div>


              <div style={{ width: '2%' }}>
                    {}
              </div>



              <div style={{ width: "25%"}}>

                <Section title={"Servo Device"}>

                    {this.renderDeviceSelection()}


                          {(device_selected === true) ?
                          <NepiDeviceSVXControls
                              namespace={namespace}
                              make_section={false}
                        />
                        : null}

                        
                    </Section>

                      {(device_selected === true) ?
                      <NepiIFSettings
                        settingsNamespace={namespace + '/settings'}
                        allways_show_settings={true}
                        make_section={true}
                        title={"Device Settings"}
                    />
                    : null}



                    {(device_selected === true) ?
                      <NepiIFAdmin
                          title={"Advanced Settings"}
                          show_advanced_option={true}
                          show_admin_device_names={true}
                          node_name={node_name}
                          make_section={true}
                    />
                    : null}

              </div>

        </div>


          </Column>
        </Columns>

        )
  }

}


export default NepiDeviceSVX
