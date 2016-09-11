!/bin/bash
cd ~
git clone https://github.com/IntelRealSense/librealsense.git
sudo apt-get update && sudo apt-get upgrade
sudo apt-get install libusb-1.0-0-dev
cd librealsense
./scripts/install_glfw3.sh
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger
./scripts/patch-uvcvideo-ubuntu-mainline.sh
sudo modprobe uvcvideo
make && sudo make install
echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib" >> ~/.bashrc
cd ~/catkin_ws/src
git clone https://github.com/intel-ros/realsense.git
cd realsense
git checkout stable
rosdep install --skip-keys=librealsense --from-paths -i . 
cd ~/catkin_ws
sudo apt-get -y install ros-indigo-realsense-camera
catkin_make

