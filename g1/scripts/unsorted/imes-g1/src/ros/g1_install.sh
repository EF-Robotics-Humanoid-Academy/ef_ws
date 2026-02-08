#!/usr/bin/env bash

function color_echo () {
    echo "$(tput setaf 2; tput bold)$1$(tput sgr0)"
}

function install_binary_packages () {
    color_echo "Installing build pacakges."
    sudo apt-get install libpcl-dev build-essential\
                         neofetch\
                         cmake\
                         libglfw3-dev\
                         libglew-dev\
                         libeigen3-dev\
                         libjsoncpp-dev\
                         libtclap-dev\
                         libeigen3-dev\
                         libboost-all-dev\
                         gstreamer1.0-plugins-base\
                         gstreamer1.0-plugins-good\
                         gstreamer1.0-plugins-bad\
                         gstreamer1.0-plugins-ugly\
                         gstreamer1.0-libav\
                         gstreamer1.0-doc\
                         gstreamer1.0-tools\
                         gstreamer1.0-x\
                         gstreamer1.0-alsa\
                         gstreamer1.0-gl\
                         gstreamer1.0-gtk3\
                         gstreamer1.0-qt5\
                         gstreamer1.0-pulseaudio\
                         libgstreamer1.0-dev\
                         libgstreamer-plugins-base1.0-dev\
                         gstreamer1.0-tools\
                         libavcodec-dev\
                         libavformat-dev\
                         libavutil-dev\
                         libswscale-dev\
                         libavresample-dev\
                         libgtk-3-dev\
                         git\
                         libbullet-dev \
                         python3-colcon-common-extensions \
                         python3-flake8 \
                         python3-pip \
                         python3-pytest-cov \
                         python3-rosdep \
                         python3-setuptools \
                         python3-vcstool \
                         wget \
                         nano \
                         clang-format-10 -y

    color_echo "Installing FAST LIO Dependencies."
    sudo apt install libc++-dev libc++abi-dev -y
    sudo apt-get install libeigen3-dev -y

    pip3 install flask

    color_echo "Installing build pacakges."
    sudo apt install ros-dev-tools
    sudo apt-get install ros-foxy-robot-upstart\
                         ros-foxy-xacro\
                         ros-foxy-slam-toolbox\
                         ros-foxy-teleop-twist-keyboard\
                         ros-foxy-teleop-twist-joy\
                         ros-foxy-geodesy\
                         ros-foxy-pcl-ros\
                         ros-foxy-nmea-msgs\
                         ros-foxy-robot-localization\
                         ros-foxy-interactive-marker-twist-server\
                         ros-foxy-pointcloud-to-laserscan\
                         ros-foxy-twist-mux\
                         ros-foxy-rmw-cyclonedds-cpp\
                         ros-foxy-rosidl-generator-dds-idl\
                         ros-foxy-navigation2\
                         ros-foxy-moveit\
                         ros-foxy-joint-state-publisher-gui\
                         ros-foxy-ros2-control\
                         ros-foxy-ros2-controllers\
                         ros-foxy-gripper-controllers\
                         ros-foxy-test-msgs\
                         ros-foxy-moveit\
                         ros-foxy-moveit-servo\
                         ros-foxy-navigation2\
                         ros-foxy-hardware-interface\
                         ros-foxy-realsense2-*\
                         ros-foxy-rqt-tf-tree\
                         ros-foxy-librealsense2*\
                         ros-foxy-rosbridge-server\
                         ros-foxy-nav2-* -y
}

function install_debian () {
    color_echo "Copying over debian packages"
    sudo cp g1_bringup/debian/45-mbs.rules /etc/udev/rules.d/
    sudo service udev restart && sudo udevadm trigger
    sudo usermod -aG input $USER
}

function install_motd () {
    color_echo "Copying over motd"
    sudo chmod -x /etc/update-motd.d/*
    sudo cp g1_bringup/config/10-qre-g1-motd /etc/update-motd.d/
    sudo chmod +x /etc/update-motd.d/10-qre-g1-motd
}

function install_livox () {
    read -p "Do you want to install Livox SDK? (y/n): " answer
    if [[ "$answer" == "y" || "$answer" == "Y" ]]; then
        color_echo "Installing Livox SDK"
        cd ../third_party/sensors/05Jun2025_Livox-SDK2
        mkdir build
        cd build
        cmake .. && make -j
        sudo make install
    else
        echo "Skipping Livox SDK installation."
    fi
}


RED='\033[0;31m'
DGREEN='\033[0;32m'
GREEN='\033[1;32m'
WHITE='\033[0;37m'
BLUE='\033[1;34m'
CYAN='\033[1;36m'
NC='\033[0m' 
                                                                                          
echo -e "${DGREEN}------------------------------------------------------------------------------------------------------"
echo -e " _______           _______  ______   _______           _______  _______  ______     ______   _______  "
echo -e "(  ___  )|\     /|(  ___  )(  __  \ (  ____ )|\     /|(  ____ )(  ____ \(  __  \   (  __  \ (  ____ \ "
echo -e "| (   ) || )   ( || (   ) || (  \  )| (    )|| )   ( || (    )|| (    \/| (  \  )  | (  \  )| (    \/ "
echo -e "| |   | || |   | || (___) || |   ) || (____)|| |   | || (____)|| (__    | |   ) |  | |   ) || (__     "
echo -e "| |   | || |   | ||  ___  || |   | ||     __)| |   | ||  _____)|  __)   | |   | |  | |   | ||  __)    "
echo -e "| | /\| || |   | || (   ) || |   ) || (\ (   | |   | || (      | (      | |   ) |  | |   ) || (       "
echo -e "| (_\ \ || (___) || )   ( || (__/  )| ) \ \__| (___) || )      | (____/\| (__/  )_ | (__/  )| (____/\ "
echo -e "(____\/_)(_______)|/     \|(______/ |/   \__/(_______)|/       (_______/(______/(_)(______/ (_______/ "
echo -e ""                                                                                                                                   
echo -e "------------------------------------------------------------------------------------------------------"
echo -e "Installing Required Libraries and ROS dependencies! "                                                                                                                                         
echo -e "------------------------------------------------------------------------------------------------------${NC}"

# Binary packages installation
install_binary_packages
install_motd
install_debian
install_livox