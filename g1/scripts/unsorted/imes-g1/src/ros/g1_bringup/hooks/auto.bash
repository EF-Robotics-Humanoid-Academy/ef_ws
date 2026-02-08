#!/bin/bash

CYAN='\033[0;36m'

source  /opt/ros/foxy/setup.bash;
export G1_NS="g1_unit_5849";
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp;
#export CYCLONEDDS_URI=/opt/mybotshop/src/mybotshop/g1_bringup/config/def_cyclone.xml;

# export ROS_DOMAIN_ID=10

echo -e "${CYAN}MYBOTSHOP G1 Environment Setup Complete"
