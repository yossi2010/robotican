//
// Created by tom on 14/04/16.
//

#ifndef ROBOTICAN_HARDWARE_INTERFACE_LIZIROBOT_H
#define ROBOTICAN_HARDWARE_INTERFACE_LIZIROBOT_H

#include <ric_board/Motor.h>
#include <robotican_hardware_interface/robot_base.h>
#include <hardware_interface/joint_command_interface.h>
#include <robotican_hardware_interface/dynamixel_pro_controller.h>
#include <dynamixel_msgs/JointState.h>

namespace robotican_hardware {

    typedef std::pair<std::string, JointInfo_t> joint_pair;

    class LiziRobot : public RobotBase {
    private:
        hardware_interface::PositionJointInterface _posJointInterface;
        hardware_interface::JointStateInterface _jointStateInterface;
        hardware_interface::VelocityJointInterface _velocityJointInterface;
        dynamixel_pro_controller::DynamixelProController* _dynamixelProController;

        joint_pair _rearLeftMotorJointInfo;
        joint_pair _rearRightMotorJointInfo;
        joint_pair _frontLeftMotorJointInfo;
        joint_pair _frontRightMotorJointInfo;

        joint_pair _panInfo;
        joint_pair _tiltInfo;

        bool _havePanTilt;

    protected:

    public:
        LiziRobot();
        virtual ~LiziRobot();
        virtual void registerInterfaces();
        virtual void read();
        virtual void write();
    };
}

#endif //ROBOTICAN_HARDWARE_INTERFACE_LIZIROBOT_H
