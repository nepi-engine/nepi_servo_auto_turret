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

import { Columns, Column } from "./Columns"
import Select, { Option } from "./Select"
import Styles from "./Styles"
import Button, { ButtonMenu } from "./Button"


import NepiIFApps from "./Nepi_IF_Apps"


// DEVICE Classes
import DriversMgr from "./NepiSystemDrivers"
import IDX from "./NepiDeviceIDX"
import PTX from "./NepiDevicePTX"
import LSX from "./NepiDeviceLSX"
import RBX from "./NepiDeviceRBX"
import NPX from "./NepiDeviceNPX"
import SVX from "./NepiDeviceSVX"


// DATA Classes



// PROCESS Classes
import ScriptsMgr from "./NepiMgrScripts"

// AUTO CLASSES


// Other SYSTEM Classes
import DeviceMgr from "./NepiSystemDevice"
import NepiMgr from "./NepiSystemAdmin"
import SystemMgr from "./NepiSystemSetup"
import DataMgr from "./NepiSystemData"
import SoftwareMgr from "./NepiSystemSoftware"
import AppsMgr from "./NepiSystemApps"
import NavPoseMgr from "./NepiSystemNavPose"
import AiModelsMgr from "./NepiSystemAiModels"

import AiDetectorMgr from "./NepiMgrAiDetector"
//import AiSegmentorMgr from "./NepiMgrAiSegmentor"
//import AiPoserMgr from "./NepiMgrAiPoser"
//import AiOrientatorMgr from "./NepiMgrAiOrientator"
//import TargetsMgr from "./NepiMgrTargets"





@inject("ros")
@observer

class NepiIFAppSelector extends Component {
  constructor(props) {
    super(props)

    this.state = {

      connectedToNepi: false,
      connectedToAppsMgr: false,
      connectedToDriversMgr: false,
      connectedToAiModelsMgr: false,
      app_id: 'None',
      selected_app: 'None',
      full_screen: false,

      needs_update: false
    }


    this.checkConnection = this.checkConnection.bind(this)
    this.getAppOptions = this.getAppOptions.bind(this)
    this.onToggleAppSelection = this.onToggleAppSelection.bind(this)  

    
  }



  async checkConnection() {
    const { connectedToNepi , connectedToAppsMgr, connectedToDriversMgr, connectedToAiModelsMgr} = this.props.ros
    if (this.state.connectedToNepi !== connectedToNepi){
      this.setState({connectedToNepi: connectedToNepi,
                    app_id: 'None',
                    selected_app: 'None', needs_update: true})
    }
    if (this.state.connectedToAppsMgr !== connectedToAppsMgr  || 
         this.state.connectedToDriversMgr !== connectedToDriversMgr || 
         this.state.connectedToAiModelsMgr !== connectedToAiModelsMgr)
    {
      this.setState({needs_update: true})
    }

    setTimeout(async () => {
      await this.checkConnection()
    }, 100)
  }




  componentDidMount(){
    this.checkConnection()
  }

  // Lifecycle method called when compnent updates.
  // Used to track changes in the topic
  componentDidUpdate(prevProps, prevState, snapshot) {
    const needs_update = this.state.needs_update
    if (needs_update) {
      this.setState({ needs_update: false})
    }
  }





  // Lifecycle method called just before the component umounts.
  // Used to unsubscribe to Status message
  componentWillUnmount() {
      this.setState({selected_app: 'None'})
  }





  renderApplication() {
    const sel_app = this.state.selected_app
    const {apps_list} = this.props.ros

    if (sel_app === "None"){
      return (
        <React.Fragment>
      <Columns>
        <Column>

      </Column>
      </Columns>   
        </React.Fragment>
      )
    }
    else if (sel_app === "Drivers Manager"){
      return (
        <React.Fragment>
          <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
            {sel_app}
          </label>
      <Columns>
        <Column>
        <DriversMgr
         title={"Drivers Manager"}
         />
      </Column>
      </Columns> 
        </React.Fragment>
      )
    }
    else if (sel_app === "Imaging"){
      return (
        <React.Fragment>
            <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
            {this.state.selected_app}
            </label>
      <Columns>
        <Column>
        <IDX
         title={"IdxDevice"}
         />
      </Column>
      </Columns>    
        </React.Fragment>
      )
    }
    else if (sel_app === "PanTilts"){
      return (
        <React.Fragment>
          <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
            {this.state.selected_app}
          </label>
      <Columns>
        <Column>
        <PTX
         title={"PtxDevice"}
         />
      </Column>
      </Columns>   
        </React.Fragment>
      )
    }
    else if (sel_app === "Lights"){
      return (
        <React.Fragment>
          <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
            {this.state.selected_app}
          </label>
        <Columns>
        <Column>
        <LSX
         title={"LsxDevice"}
         />
      </Column>
      </Columns>   
        </React.Fragment>
      )
    }
    else if (sel_app === "Robots"){
      return (
        <React.Fragment>
          <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
            {this.state.selected_app}
          </label>
      <Columns>
        <Column>
        <RBX
         title={"RbxDevice"}
         />
         </Column>
         </Columns>   
        </React.Fragment>
      )
    }
    else if (sel_app === "NavPose"){
      return (
        <React.Fragment>
          <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
            {this.state.selected_app}
          </label>
      <Columns>
        <Column>
        <NPX
         title={"NpxDevice"}
         />
      </Column>
      </Columns>
        </React.Fragment>
      )
    }
    else if (sel_app === "Servos"){
      return (
        <React.Fragment>
          <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
            {this.state.selected_app}
          </label>
      <Columns>
        <Column>
        <SVX
         title={"SvxDevice"}
         />
      </Column>
      </Columns>
        </React.Fragment>
      )
    }




    else if (sel_app === "NavPose Manager"){
      return (
        <React.Fragment>
            <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
            {sel_app}
            </label>
            <Columns>
            <Column>

              <NavPoseMgr
              title={"NavPose Manager"}
              />

          </Column>
          </Columns>  
        </React.Fragment>
      )
    }

    else if (sel_app === "Data Manager"){
      return (
        <React.Fragment>
            <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
            {sel_app}
            </label>
            <Columns>
            <Column>

              <DataMgr
              title={"Data Manager"}
              />

          </Column>
          </Columns>  
        </React.Fragment>
      )
    }


    // if (sel_app === "Targeting"){
    //   return (
    //     <React.Fragment>
    //         <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
    //         {sel_app}
    //         </label>
    //         <Columns>
    //         <Column>

    //           <TargetsMgr
    //           title={"Targeting"}
    //           />

    //       </Column>
    //       </Columns>  
    //     </React.Fragment>
    //   )
    // }
    if (sel_app === "AI Detector"){
      return (
        <React.Fragment>
            <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
            {sel_app}
            </label>
            <Columns>
            <Column>

              <AiDetectorMgr
              title={"AI Detector"}
              />

          </Column>
          </Columns>  
        </React.Fragment>
      )
    }

    /*
    if (sel_app === "AI Segmentor"){
      return (
        <React.Fragment>
            <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
            {sel_app}
            </label>
            <Columns>
            <Column>

              <AiSegmentorMgr
              title={"AI Segmentor"}
              />

          </Column>
          </Columns>  
        </React.Fragment>
      )
    }
    if (sel_app === "AI Poser"){
      return (
        <React.Fragment>
            <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
            {sel_app}
            </label>
            <Columns>
            <Column>

              <AiPoserMgr
              title={"AI Poser"}
              />

          </Column>
          </Columns>  
        </React.Fragment>
      )
    }
    if (sel_app === "AI Orientator"){
      return (
        <React.Fragment>
            <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
            {sel_app}
            </label>
            <Columns>
            <Column>

              <AiOrientatorMgr
              title={"AI Orientator"}
              />

          </Column>
          </Columns>  
        </React.Fragment>
      )
    }
  */
    if (sel_app === "AI Model Manager"){
      return (
        <React.Fragment>
            <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
            {sel_app}
            </label>
            <Columns>
            <Column>

              <AiModelsMgr
              title={"AI Model Manager"}
              />

          </Column>
          </Columns>  
        </React.Fragment>
      )
    }


    else if (sel_app === "Device Manager"){
      return (
        <React.Fragment>
            <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
            {sel_app}
            </label>
            <Columns>
            <Column>

              <DeviceMgr
              title={"Device Manager"}
              />

          </Column>
          </Columns>  
        </React.Fragment>
      )
    }
    else if (sel_app === "NEPI Manager"){
      return (
        <React.Fragment>
            <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
            {sel_app}
            </label>
            <Columns>
            <Column>

              <NepiMgr
              title={"NEPI Manager"}
              />

          </Column>
          </Columns>  
        </React.Fragment>
      )
    }
    else if (sel_app === "System Manager"){
      return (
        <React.Fragment>
            <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
            {sel_app}
            </label>
            <Columns>
            <Column>

              <SystemMgr
              title={"System Manager"}
              />

          </Column>
          </Columns>  
        </React.Fragment>
      )
    }
    else if (sel_app === "Scripts Mgr"){
      return (
        <React.Fragment>
            <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
            {sel_app}
            </label>
            <Columns>
            <Column>

              <ScriptsMgr
              title={"Scripts Manager"}
              />

          </Column>
          </Columns>  
        </React.Fragment>
      )
    }

    else if (sel_app === "Software Manager"){
      return (
        <React.Fragment>
            <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
            {sel_app}
            </label>
            <Columns>
            <Column>

              <SoftwareMgr
              title={"Software Manager"}
              />

          </Column>
          </Columns>  
        </React.Fragment>
      )
    }
    if (sel_app === "Apps Manager"){
      return (
        <React.Fragment>
            <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
            {sel_app}
            </label>
            <Columns>
            <Column>

              <AppsMgr
              title={"Apps Manager"}
              />

          </Column>
          </Columns>  
        </React.Fragment>
      )
    }



    else if (apps_list.indexOf(sel_app) !== -1){
      return (
         <NepiIFApps
         sel_app={sel_app}
         />
       );
     }


    else {
      if (sel_app !== 'None'){
        this.setState({selected_app: 'None'})
      }
      return (
        <React.Fragment>
            {/* <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
            {sel_app}
          </label> */}

          <Columns>
          <Column>

          </Column>
          </Columns> 
        </React.Fragment>
      )
    }


  }




  onToggleAppSelection(event){
    const app_name = event.target.value
    if (app_name === 'Connecting' || app_name === 'None'){
      this.setState({full_screen: false })
    }
    else {
      this.setState({selected_app: app_name, full_screen: true })
    }
  }


  // Function for creating image topic options.
  getAppOptions() {
    const {idxDevices,lsxDevices,ptxDevices,rbxDevices,npxDevices,svxDevices} = this.props.ros
    
    const app_id = (this.props.app_id !== undefined) ? this.props.app_id : 'None'

    const connected = this.state.connectedToNepi

    const appsList = this.props.ros.apps_list
    const nameList = this.props.ros.apps_name_list 

    const groupList = this.props.ros.apps_group_list
    const runningList = this.props.ros.apps_running_list

    const managers_running = this.props.ros.managers_running_list
    const restricted = this.props.ros.userRestricted

    const activeModelTypes = this.props.ros.ai_models_running_type_list

    var items = []
    if (connected !== true){
      items.push(<Option value={'Connecting'}>{'Connecting'}</Option>)
    }
    else {
      if (app_id === 'DEVICE') {

        if (Object.keys(idxDevices).length > 0 && restricted.indexOf('DEVICE-IDX-VIEW') === -1){
          items.push(<Option value={"Imaging"}>{"Imaging"}</Option>)
        }
        if (Object.keys(ptxDevices).length > 0 && restricted.indexOf('DEVICE-PTX-VIEW') === -1){
          items.push(<Option value={"PanTilts"}>{"PanTilts"}</Option>)
        }
        if (Object.keys(lsxDevices).length > 0 && restricted.indexOf('DEVICE-LSX-VIEW') === -1){
          items.push(<Option value={"Lights"}>{"Lights"}</Option>)
        }
        if (Object.keys(rbxDevices).length > 0 && restricted.indexOf('DEVICE-RBX-VIEW') === -1){
          items.push(<Option value={"Robots"}>{"Robots"}</Option>)
        }
        if (Object.keys(npxDevices).length > 0 && restricted.indexOf('DEVICE-NPX-VIEW') === -1){
          items.push(<Option value={"NavPose"}>{"NavPose"}</Option>)
        }
        if (Object.keys(svxDevices).length > 0 && restricted.indexOf('DEVICE-SVX-VIEW') === -1){
          items.push(<Option value={"Servos"}>{"Servos"}</Option>)
        }
  

        if (appsList.length > 0){
          for (var i1 = 0; i1 < appsList.length; i1++) {
            if (groupList[i1] === "DEVICE" && appsList[i1] !== "None" && runningList.indexOf(appsList[i1]) !== -1){
              items.push(<Option value={appsList[i1]}>{nameList[i1]}</Option>)
            }
          }
        }
      }
      else if (app_id === 'DATA') {
        if (appsList.length > 0){
          for (var i2 = 0; i2 < appsList.length; i2++) {
            if (groupList[i2] === "DATA" && appsList[i2] !== "None" && runningList.indexOf(appsList[i2]) !== -1 ){
              items.push(<Option value={appsList[i2]}>{nameList[i2]}</Option>)
            }
          }
        }
      }

      else if (app_id === 'PROCESS') {
        
        if (appsList.length > 0){
          for (var i3 = 0; i3 < appsList.length; i3++) {
            if (groupList[i3] === "PROCESS" && appsList[i3] !== "None" && runningList.indexOf(appsList[i3]) !== -1){
              items.push(<Option value={appsList[i3]}>{nameList[i3]}</Option>)
            }
          }
        }

        if (true) { 
            

            if (activeModelTypes.indexOf('detection') !== -1 && restricted.indexOf('MANAGER-AI-DETECTORS-VIEW') === -1){
              items.push(<Option value={'AI Detector'}>{'AI Detector'}</Option>)
            }
            if (activeModelTypes.indexOf('segmentation') !== -1 && restricted.indexOf('MANAGER-AI-DETECTORS-VIEW') === -1){
              items.push(<Option value={'AI Segmetation'}>{'AI Segmetation'}</Option>)
            }
            if (activeModelTypes.indexOf('pose') !== -1 && restricted.indexOf('MANAGER-AI-DETECTORS-VIEW') === -1){
              items.push(<Option value={'AI Pose'}>{'AI Pose'}</Option>)
            }
            if (activeModelTypes.indexOf('orientation') !== -1 && restricted.indexOf('MANAGER-AI-DETECTORS-VIEW') === -1){
              items.push(<Option value={'AI Orienation'}>{'AI Orienation'}</Option>)
            }
        }
      }

      else if (app_id === 'AUTOMATION') {
        if (appsList.length > 0){
          for (var i4 = 0; i4 < appsList.length; i4++) {
            if (groupList[i4] === "AUTOMATION" && appsList[i4] !== "None" && runningList.indexOf(appsList[i4]) !== -1){
              items.push(<Option value={appsList[i4]}>{nameList[i4]}</Option>)
            }
          }
        }
      }


      else if (app_id === 'SYSTEM') {

        if (restricted.indexOf('MANAGER-DEVICE-VIEW') === -1) { 
            items.push(<Option value={'Device Manager'}>{'Device Manager'}</Option>)
        }   
        
        if (restricted.indexOf('MANAGER-NEPI-VIEW') === -1) { 
            items.push(<Option value={'NEPI Manager'}>{'NEPI Manager'}</Option>)
        }   

        if (restricted.indexOf('MANAGER-SYSTEM-VIEW') === -1) { 
            items.push(<Option value={'System Manager'}>{'System Manager'}</Option>)
        }   

        if (restricted.indexOf('MANAGER-DATA-VIEW') === -1) { 
          items.push(<Option value={"Data Manager"}>{"Data Manager"}</Option>)
        }

        if (restricted.indexOf('MANAGER-SOFTWARE-VIEW') === -1 && managers_running.indexOf('MANAGER-SOFTWARE') !== -1 ) {
            items.push(<Option value={'Software Manager'}>{'Software Manager'}</Option>)
        }        
        

        if (restricted.indexOf('MANAGER-NAVPOSE-VIEW') === -1 && managers_running.indexOf('MANAGER-NAVPOSE') !== -1 ) {
            items.push(<Option value={"NavPose Manager"}>{"NavPose Manager"}</Option>)
        }


        if (restricted.indexOf('MANAGER-DRIVERS-VIEW') === -1 && managers_running.indexOf('MANAGER-DRIVERS') !== -1 ) {
            items.push(<Option value={'Drivers Manager'}>{'Drivers Manager'}</Option>)
        }   
        
        if (restricted.indexOf('MANAGER-APPS-VIEW') === -1 && managers_running.indexOf('MANAGER-APPS') !== -1 ) {
           items.push(<Option value={'Apps Manager'}>{'Apps Manager'}</Option>)
        }   

        if (restricted.indexOf('MANAGER-SCRIPTS-VIEW') === -1 && managers_running.indexOf('MANAGER-SCRIPTS') !== -1 ) {
           items.push(<Option value={'Scripts Mgr'}>{'Scripts Manager'}</Option>)
        }
        
        if (restricted.indexOf('MANAGER-AI_MODELS-VIEW') === -1 && managers_running.indexOf('MANAGER-AI-MODELS') !== -1 ) {
           items.push(<Option value={'AI Model Manager'}>{'AI Model Manager'}</Option>)
        

        if (activeModelTypes.indexOf('detection') !== -1 && restricted.indexOf('MANAGER-AI-DETECTORS-VIEW') === -1){
          items.push(<Option value={'AI Detector'}>{'AI Detector'}</Option>)
        }
        if (activeModelTypes.indexOf('segmentation') !== -1 && restricted.indexOf('MANAGER-AI-SEGMENTATION-VIEW') === -1){
          items.push(<Option value={'AI Segmetation'}>{'AI Segmetation'}</Option>)
        }
        if (activeModelTypes.indexOf('pose') !== -1 && restricted.indexOf('MANAGER-AI-POSE-VIEW') === -1){
          items.push(<Option value={'AI Pose'}>{'AI Pose'}</Option>)
        }
        if (activeModelTypes.indexOf('orientation') !== -1 && restricted.indexOf('MANAGER-AI-ORIENTATION-VIEW') === -1){
          items.push(<Option value={'AI Orienation'}>{'AI Orienation'}</Option>)
        }        


        }   
        
 
        if (appsList.length > 0){
          for (var i5 = 0; i5 < appsList.length; i5++) {
            if (groupList[i5] === "SYSTEM" && appsList[i5] !== "None" && runningList.indexOf(appsList[i5]) !== -1){
              items.push(<Option value={appsList[i5]}>{nameList[i5]}</Option>)
            }
          }
        }
      }
    }

    if (items.length === 0){
      items.push(<Option value={'None'}>{'Waiting for Apps'}</Option>)
    }


    return items
  }


    renderSelection() {
    const app_options = this.getAppOptions()

    return (
      <React.Fragment>

      <Columns>
        <Column>

        <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
          {"Select App"}
         </label>


        <div style={{backgroundColor: Styles.vars.colors.grey0}}>
            <Select style={{width: "10px"}}/>
            {app_options.map((app) =>
            <div onClick={this.onToggleAppSelection}
              style={{
                textAlign: "center",
                padding: `${Styles.vars.spacing.xs}`,
                color: Styles.vars.colors.black,
                backgroundColor: (app.props.value === this.state.selected_app) ? Styles.vars.colors.blue : Styles.vars.colors.grey0,
                cursor: "pointer",
                }}>
                <body app-topic ={app} style={{color: (app === 'Connecting') ? Styles.vars.colors.blue : Styles.vars.colors.black}}>{app}</body>
            </div>
            )}
         
        </div>
      </Column>
      </Columns>

      </React.Fragment>
    )
  }



  render() {
    const app_id = (this.props.app_id !== undefined) ? this.props.app_id : 'None'
    if (this.state.app_id !== app_id){
      this.setState({app_id: app_id,
                    selected_app: 'None', needs_update: true})

    }
    const app_selected = (this.state.selected_app !== 'None')
    const full_screen = (this.state.full_screen === true) && (app_selected === true)
    const sel_col_width = (full_screen === false) ? '12%' : '3%'
    const app_col_width = (full_screen === false) ? '85%' : '94%'

        
      return (
        <React.Fragment>

        <div style={{ display: 'flex' }}>
          <div style={{ width: sel_col_width }}>


              { (full_screen === true) ?

                        <ButtonMenu>
                          <Button onClick={() => this.setState({full_screen: false})}>{'\u25BC'}</Button>
                        </ButtonMenu>

                    
                :
                  this.renderSelection()
              }

          </div>

          <div style={{ width: '3%' }}>
              {}
          </div>

          <div style={{ width: app_col_width }}>

              
              { (app_selected === true) ?
                this.renderApplication()
                : null }

          </div>
        </div>
        
        </React.Fragment>

      )
  }

}

export default NepiIFAppSelector
