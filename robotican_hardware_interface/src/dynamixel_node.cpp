//
// Created by tom on 29/11/16.
//

#include <ros/ros.h>
#include <robotican_hardware_interface/dynamixel_controller.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "dynamixel_node");
    ros::Rate loopRate(50);
    ros::AsyncSpinner spinner(2);
    ros::NodeHandle handle;
    dynamixel_controller::DynamixelController controller;

    spinner.start();
    ROS_INFO("ARM active");
    while (ros::ok()) {
        controller.read();
        controller.publishState();
        controller.write();
        loopRate.sleep();
    }

    return 0;

}