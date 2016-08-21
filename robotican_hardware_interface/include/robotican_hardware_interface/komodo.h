//
// Created by tom on 17/04/16.
//

#ifndef ROBOTICAN_HARDWARE_INTERFACE_KOMODO_H
#define ROBOTICAN_HARDWARE_INTERFACE_KOMODO_H

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <dynamixel_msgs/JointState.h>
#include <robotican_hardware_interface/robot_base.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/posvel_command_interface.h>
#include <robotican_hardware_interface/dynamixel_pro_controller.h>

namespace robotican_hardware {
    class KomodoRobot : public RobotBase {
    private:
        bool _first[2];

        hardware_interface::PositionJointInterface _positionJointInterface;
        hardware_interface::PosVelJointInterface _posVelJointInterface;
        dynamixel_pro_controller::DynamixelProController* _dynamixelProController;
        std::pair<std::string, JointInfo_t> _leftFingerInfo;
        std::pair<std::string, JointInfo_t> _rightFingerInfo;

        ros::Publisher _leftFingerCmd;
        ros::Publisher _rightFingerCmd;

        ros::Subscriber _leftFingerState;
        ros::Subscriber _rightFingerState;

        void leftFingerCallback(const dynamixel_msgs::JointState::ConstPtr &msg);
        void rightFingerCallback(const dynamixel_msgs::JointState::ConstPtr &msg);

    public:
        KomodoRobot();
        virtual ~KomodoRobot();
        virtual void registerInterfaces();
        virtual void read();
        virtual void write();
    };
}


#endif //ROBOTICAN_HARDWARE_INTERFACE_KOMODO_H
