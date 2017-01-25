#!/bin/bash

cd ~/catkin_ws/src
if [ $? == 0 ]; then
	
	#installing robotican pkg depends
	rosdep update
	sudo apt-get update
	sudo apt-get dist-upgrade
	sudo apt-get -f dist-upgrade
	sudo apt-get install python-wstool python-catkin-tools clang-format-3.8
	sudo apt-get -f install ros-indigo-moveit

	sudo apt-get install ros-indigo-ar-track-alvar
	sudo apt-get install ros-indigo-move-base 
	sudo apt-get install ros-indigo-controller-manager
	sudo apt-get install ros-indigo-gripper-action-controller 
	sudo apt-get install ros-indigo-joint-trajectory-controller
	sudo apt-get install ros-indigo-transmission-interface 
	sudo apt-get install ros-indigo-joint-limits-interface 
	sudo apt-get install ros-indigo-gazebo-ros-control
	sudo apt-get install ros-indigo-serial

	#Third party apt-get pkgs
	cd ~/catkin_ws/src/robotican/robotican/installations/third_pkg_setup
	sudo ./setup.sh
	
	#USB rules setup
	cd ~/catkin_ws/src/robotican/robotican/installations/usb_rules_setup
	sudo ./setup.sh
	
	#Changing to user.
	sudo chown -R $(logname):$(logname) ~/catkin_ws
	
	echo "Do you want to install kinect2 package [y/n]: "
	read asfK
	if [ $asfK == "y" ]; then
		cd ~/catkin_ws/src/robotican/robotican/installations/third_pkg_setup
		./kinect2.sh
	fi
	
	echo "Do you want to install softkinetic (DS325) package [y/n]: "
	read asfS
	if [ $asfS == "y" ]; then
		cd ~/catkin_ws/src/robotican/robotican/installations/third_pkg_setup
		sudo ./softkinetic.sh
	fi
	
	cd ~/catkin_ws
	catkin_make

	#Do this in the end of the installation.
	if [ $asf == "y" ]; then
		echo -e "\e[31mWarning: To complete f200\r200\sr300\softkinetic installation the PC need to reboot."
		echo -en "\e[39mReboot the PC [y/n]:"
		read as

		if [ $as == "y" ]; then
			sudo reboot
		fi
	fi

else
	echo -e "\e[31m[Error]: ROS catkin_ws not found"
	echo -en "\e[39m"
	exit 1
fi

exit
#END
