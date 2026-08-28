#!/bin/bash
##
## Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
##
## This file is part of nepi-engine
## (see https://github.com/nepi-engine).
##
## License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
##

#######################################################################################################
# Usage: $ ./deploy_nepi_engine_complete.sh
#
# This script copies the complete nepi_engine source code to proper filesystem locations on target
# hardware in preparation for building nepi-engine from source. 
#
# It can be run from a development host or directly on the target hardware as described in this
# repository's README
#
# The script requires the following environment variable be set
#    NEPI_REMOTE_SETUP: Indicates whether running from development host or directly on target 
#                      (1 = Dev. Host, 0 = From Target)
# In the case that NEPI_REMOTE_SETUP == 1, some further environment variables must be set
#    NEPI_TARGET_IP: Target IP address/hostname
     NEPI_TARGET_IP=${NEPI_IP} #/${NEPI_DEVICE_ID}
     echo "Using target IP: ${NEPI_TARGET_IP}"
#    NEPI_DEPLOY_USERNAME: Target username

     NEPI_DEPLOY_USERNAME=nepihost
     NEPI_SSH_PORT=22
#    NEPI_SSH_KEY: Private SSH key for SSH/Rsync to target (as applicable)
     NEPI_SSH_KEY=${HOME}/.ssh/nepi_default_ssh_key
#    NEPI_TARGET_SRC_DIR: Directory to deploy source code to
     NEPI_TARGET_SRC_DIR=/mnt/nepi_storage/nepi_src
#    NEPI_SETUP_SRC_DIR: Directory to deploy setup source to
     NEPI_SETUP_SRC_DIR=/home/nepihost
#######################################################################################################
# # Clear known hosts keys
# sudo rm /home/${USER}/.ssh/known*
########################################
sudo -v


REPO="REPO_FOLDER_NAME" 


# Set NEPI folder variables if not configured by nepi aliases bash script
if [[ -z "${NEPI_USER+x}" ]]; then
    NEPI_USER=nepi
fi
if [[ -z "${NEPI_HOME+x}" ]]; then
    NEPI_HOME=/home/${NEPI_USER}
fi
if [[ -z "${NEPI_DOCKER+x}" ]]; then
    NEPI_DOCKER=/mnt/nepi_docker
fi
if [[ -z "${NEPI_STORAGE+x}" ]]; then
   NEPI_STORAGE=/mnt/nepi_storage
fi

if [[ -z "${NEPI_CONFIG+x}" ]]; then
    NEPI_CONFIG=/mnt/nepi_config
fi
if [[ -z "${NEPI_BASE+x}" ]]; then
    NEPI_BASE=/opt/nepi
fi
if [[ -z "${NEPI_RUI+x}" ]]; then
    NEPI_RUI=${NEPI_BASE}/nepi_rui
fi
if [[ -z "${NEPI_ENGINE+x}" ]]; then
    NEPI_ENGINE=${NEPI_BASE}/nepi_engine
fi
if [[ -z "${NEPI_ETC+x}" ]]; then
    NEPI_ETC=${NEPI_BASE}/etc
fi


if [[ -z "${NEPI_REMOTE_SETUP}" ]]; then
  NEPI_REMOTE_SETUP=1
fi

if [ "${NEPI_REMOTE_SETUP}" == "0" ]; then
    echo "Running in Local Mode"

elif [ "${NEPI_REMOTE_SETUP}" == "1" ]; then

  if [[ -z "${NEPI_TARGET_IP}" ]]; then
    echo "Remote setup requires env. variable NEPI_TARGET_IP be assigned"
    exit 1
  fi

  if [[ -z "${NEPI_DEPLOY_USERNAME}" ]]; then
    echo "Remote setup requires env. variable NEPI_DEPLOY_USERNAME be assigned"
    exit 1
  fi
  if [[ -z "${NEPI_SSH_KEY}" ]]; then
    echo "Remote setup requires env. variable NEPI_SSH_KEY be assigned"
    exit 1
  fi
fi

####################################
# Deploy System Build and Live Files

source ./nepi_app_auto_turret/deploy_app.sh
source ./ptx_drivers/deploy_drivers.sh
source ./src/deploy_src.sh
