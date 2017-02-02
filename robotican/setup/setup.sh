#!/bin/bash

GREEN_TXT='\e[0;32m'
WHITE_TXT='\e[1;37m'
RED_TXT='\e[31m'

printf "${GREEN_TXT}\n***Installing Robotican Package***\n${WHITE_TXT}"

cd ~/catkin_ws/src
if [ $? == 0 ]; then
	
	#installing robotican pkg depends
	printf "${GREEN_TXT}Installing dependencies and 3rd party packages...\n${WHITE_TXT}"
	#--------------------------------
	
	#Fix broken packages
	sudo apt-get clean
	sudo apt-get install -f
	sudo dpkg --configure -a
	sudo apt-get update 
	
	rosdep update
	sudo apt-get update
	sudo apt-get dist-upgrade
	sudo apt-get -f dist-upgrade
	sudo apt-get install python-wstool python-catkin-tools clang-format-3.8
	
	sudo apt-get -y install ros-indigo-hector-gazebo-plugins 
	sudo apt-get -y install ros-indigo-gazebo-ros-control
	sudo apt-get -y install ros-indigo-gazebo-plugins 
	sudo apt-get -y install ros-indigo-navigation
	sudo apt-get -y install ros-indigo-move-base
	sudo apt-get -y install ros-indigo-gmapping
	sudo apt-get -fy install ros-indigo-moveit*
	sudo apt-get -fy install ros-indigo-twist-mux
	sudo apt-get -y install ros-indigo-ar-track-alvar
	sudo apt-get -y install ros-indigo-move-base 
	sudo apt-get -y install ros-indigo-controller-manager
	sudo apt-get -y install ros-indigo-gripper-action-controller 
	sudo apt-get -y install ros-indigo-joint-trajectory-controller
	sudo apt-get -y install ros-indigo-transmission-interface 
	sudo apt-get -y install ros-indigo-joint-limits-interface 
	sudo apt-get -y install ros-indigo-serial
	sudo apt-get -y install ros-indigo-hector-gazebo-plugins 
	sudo apt-get -y install ros-indigo-gazebo-plugins 
	sudo apt-get -y install ros-indigo-navigation
	sudo apt-get -y install ros-indigo-gmapping
	sudo apt-get -y install chrony openssh-server expect espeak ros-indigo-joy ros-indigo-hector-slam ros-indigo-usb-cam ros-indigo-openni2* ros-indigo-hokuyo-node  python-pygame ros-indigo-yocs-velocity-smoother ros-indigo-qt-build ros-indigo-pr2-controllers espeak espeak-data libespeak-dev
	sudo apt-get -y install ros-indigo-moveit-full ros-indigo-ueye-cam ros-indigo-gazebo-ros-pkgs ros-indigo-gazebo-ros-control ros-indigo-hector-gazebo ros-indigo-ros-controllers ros-indigo-gmapping ros-indigo-navigation ros-indigo-robot-localization ros-indigo-twist-mux ros-indigo-serial ros-indigo-ar-track-alvar mosh
	
	cd ~/catkin_ws/src
	git clone https://github.com/arebgun/dynamixel_motor.git
	git clone https://github.com/muhrix/espeak_ros.git
	
	printf "${GREEN_TXT}Done.\n\n${WHITE_TXT}"
	#--------------------------------

	#Installing USB rules 
	printf "${GREEN_TXT}Installing USB rules...\n${WHITE_TXT}"
	cd ~/catkin_ws/src/robotican/robotican/setup/usb_rules/
	sudo cp ./ric_usb.rules /etc/udev/rules.d
	sudo cp ./hokuyo.rules /etc/udev/rules.d
	sudo /etc/init.d/udev reload
	printf "${GREEN_TXT}Done.\n\n${WHITE_TXT}"
	#--------------------------------
		
	#Installing softkinetic camera
	printf "${GREEN_TXT}Installing Softkinetic camera...\n${WHITE_TXT}"
	cd ~/catkin_ws/src/robotican/robotican/setup/third_pkg
	sudo chmod +x ./DepthSenseSDK-1.9.0-5-amd64-deb.run
	./DepthSenseSDK-1.9.0-5-amd64-deb.run
	sudo echo "blacklist snd_usb_audio" >> /etc/modprobe.d/blacklist.conf
	cd ~/catkin_ws/src/
	git clone https://github.com/ipa320/softkinetic.git
	cd ~/catkin_ws
	catkin_make
	printf "${GREEN_TXT}Done.\n\n${WHITE_TXT}"
	#--------------------------------
		
	printf "${GREEN_TXT}***Installatoin Completed. Please reboot***\n${WHITE_TXT}"
	
else
	printf "${RED_TXT}[Error]: ROS catkin_ws not found\n${WHITE_TXT}"
	exit 1
fi

exit
#END
