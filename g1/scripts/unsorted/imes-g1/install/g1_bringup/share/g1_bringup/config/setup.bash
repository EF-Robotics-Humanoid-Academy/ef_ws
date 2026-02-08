#!/bin/bash

source  /opt/ros/foxy/setup.bash;
source /opt/mybotshop/install/setup.bash

alias g1_setdate='sudo date -s "$(wget --method=HEAD -qSO- --max-redirect=0 google.com 2>&1 | sed -n "s/^ *Date: *//p")"'
alias g1_build='cd /opt/mybotshop && colcon build --symlink-install && source install/setup.bash'