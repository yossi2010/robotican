//
// Created by tom on 17/04/16.
//

#ifndef ROBOTICAN_HARDWARE_INTERFACE_KOMODO_H
#define ROBOTICAN_HARDWARE_INTERFACE_KOMODO_H

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <dynamixel_msgs/JointState.h>
#include <robotican_hardware_interface/robot_base.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/posvel_command_interface.h>
#include <robotican_hardware_interface/dynamixel_controller.h>

namespace robotican_hardware {
    class KomodoRobot : public RobotBase {
    private:
        hardware_interface::PositionJointInterface _positionJointInterface;
        hardware_interface::PosVelJointInterface _posVelJointInterface;
        dynamixel_controller::DynamixelController* _dynamixelController;
        std::map<std::string, dynamixel_controller::JointInfo_t> _jointInfo;

        ros::Subscriber _armStateListener;
        ros::Publisher _armCmd;

        void armStateCallback(const sensor_msgs::JointStateConstPtr &msg);

    public:
        KomodoRobot();
        virtual ~KomodoRobot();
        virtual void registerInterfaces();
        virtual void read();
        virtual void write();
    };
}


#endif //ROBOTICAN_HARDWARE_INTERFACE_KOMODO_H
