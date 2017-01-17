#!/bin/bash

cd ~/catkin_ws/src/robotican/robotican/installations/third_pkg_setup
./DepthSenseSDK-1.9.0-5-amd64-deb.run

sudo echo "blacklist snd_usb_audio" >> /etc/modprobe.d/blacklist.conf

cd ~/catkin_ws/src/
git clone https://github.com/ipa320/softkinetic.git



echo -e "\e[34mInstalltion completed...\e[0m"
echo -en "\e[39m"

