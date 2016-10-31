//
// Created by tom on 27/10/16.
//

#include <ros/ros.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/posvel_command_interface.h>
#include <robotican_hardware_interface/dynamixel_controller.h>
#include <controller_manager/controller_manager.h>

class dynamixelTest : public hardware_interface::RobotHW {
private:
    hardware_interface::JointStateInterface _jointStateInterface;
    hardware_interface::PosVelJointInterface _posVelJointInterface;
    hardware_interface::PositionJointInterface _positionJointInterface;
    dynamixel_controller::DynamixelController _dynamixelController;

    ros::Time _time;

public:
    dynamixelTest() : _dynamixelController(&_jointStateInterface,
                                           &_posVelJointInterface,
                                           &_positionJointInterface) {
        _time = ros::Time::now();
    }
    ros::Time getTime() {
        return ros::Time::now();
    }
    ros::Duration getPeriod() {
        ros::Time now = ros::Time::now();
        ros::Duration period = now - _time;
        _time = now;
        return period;
    }

    void registerInterfaces() {
        registerInterface(&_jointStateInterface);
        registerInterface(&_posVelJointInterface);
        registerInterface(&_positionJointInterface);
    }

    void read() {
        _dynamixelController.read();
    }

    void write() {
        _dynamixelController.write();
    }
};

int main(int argc, char** argv) {

    ros::init(argc, argv, "dynamixel_test_node");
    ros::AsyncSpinner spinner(1);
    spinner.start();

//    dynamixel_driver::DynamixelDriver driver("/dev/ttyUSB0", 57142, dynamixel_driver::PROTOCOL1);
//    if(driver.isPortOpen()) {
//        dynamixel_driver::DxlMotorInfo_t motorInfo;
//        motorInfo.id = 1;
//        motorInfo.protocol = PROTOCOL1_VERSION;
//        ROS_INFO("[%s]: Port is open", ros::this_node::getName().c_str());
//        if(driver.pingMotor(motorInfo)) {
//            ROS_INFO("[%s]: Ping motor", ros::this_node::getName().c_str());
//            if(driver.setMotorTorque(motorInfo, TORQUE_DISABLE)) {
//                ROS_INFO("[%s]: Motor is torque", ros::this_node::getName().c_str());
////                if(driver.setMotorPosition(motorInfo, 4000)) {
////                    ROS_INFO("[%s]: Send to motor position cmd", ros::this_node::getName().c_str());
////                }
//            }
//        }
//    }

    dynamixelTest robot;
    robot.registerInterfaces();
    controller_manager::ControllerManager controllerManager(&robot);

    ros::AsyncSpinner asyncSpinner(2);
    asyncSpinner.start();
    ros::Rate loopRate(100);
    while (ros::ok()) {
        robot.read();
        controllerManager.update(robot.getTime(), robot.getPeriod());
        robot.write();
        loopRate.sleep();
    }
    return 0;
}


