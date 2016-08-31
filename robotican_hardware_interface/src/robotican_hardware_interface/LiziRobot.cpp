//
// Created by tom on 14/04/16.
//

#include "robotican_hardware_interface/LiziRobot.h"

namespace robotican_hardware {

    LiziRobot::LiziRobot()
    {
        std::string rearLeftJointName,
                    rearRightJointName,
                    frontRightJointName,
                    frontLeftJointName,
                    panJointName,
                    tiltJointName;

        if(!_nodeHandle.getParam("rear_left_motor_joint_name", rearLeftJointName) ||
           !_nodeHandle.getParam("rear_right_motor_joint_name", rearRightJointName) ||
            !_nodeHandle.getParam("front_right_motor_joint_name", frontLeftJointName) ||
            !_nodeHandle.getParam("front_right_motor_joint_name", frontRightJointName))
        {
            ros_utils::rosError("You must specify the following parameters: rear_left_motor_joint_name,"
                                                                            "rear_right_motor_joint_name"
                                                                            "front_right_motor_joint_name,"
                                                                            "front_right_motor_joint_name");
            ros::shutdown();
        }

        _rearLeftMotorJointInfo = joint_pair(rearLeftJointName, JointInfo_t());
        _rearRightMotorJointInfo = joint_pair(rearRightJointName, JointInfo_t());
        _frontRightMotorJointInfo = joint_pair(frontLeftJointName, JointInfo_t());
        _frontRightMotorJointInfo = joint_pair(frontRightJointName, JointInfo_t());

        hardware_interface::JointStateHandle rearLeftJointStateHandle(_rearLeftMotorJointInfo.first,
                                                                  &_rearLeftMotorJointInfo.second.position,
                                                                  &_rearLeftMotorJointInfo.second.velocity,
                                                                  &_rearLeftMotorJointInfo.second.effort);
        hardware_interface::JointStateHandle rearRightJointStateHandle(_rearRightMotorJointInfo.first,
                                                                   &_rearRightMotorJointInfo.second.position,
                                                                   &_rearRightMotorJointInfo.second.velocity,
                                                                   &_rearRightMotorJointInfo.second.effort);
        hardware_interface::JointStateHandle frontLeftJointStateHandle(_frontLeftMotorJointInfo.first,
                                                                  &_frontLeftMotorJointInfo.second.position,
                                                                  &_frontLeftMotorJointInfo.second.velocity,
                                                                  &_frontLeftMotorJointInfo.second.effort);
        hardware_interface::JointStateHandle frontRightJointStateHandle(_frontRightMotorJointInfo.first,
                                                                   &_frontRightMotorJointInfo.second.position,
                                                                   &_frontRightMotorJointInfo.second.velocity,
                                                                   &_frontRightMotorJointInfo.second.effort);

        _jointStateInterface.registerHandle(rearLeftJointStateHandle);
        _jointStateInterface.registerHandle(rearRightJointStateHandle);
        _jointStateInterface.registerHandle(frontLeftJointStateHandle);
        _jointStateInterface.registerHandle(frontRightJointStateHandle);

        hardware_interface::JointHandle rearLeftJointHandle(_jointStateInterface.getHandle(_rearLeftMotorJointInfo.first),
                                                        &_rearLeftMotorJointInfo.second.cmd);
        hardware_interface::JointHandle rearRightJointHandle(_jointStateInterface.getHandle(_rearRightMotorJointInfo.first),
                                                         &_rearRightMotorJointInfo.second.cmd);
        hardware_interface::JointHandle frontLeftJointHandle(_jointStateInterface.getHandle(_frontLeftMotorJointInfo.first),
                                                            &_frontLeftMotorJointInfo.second.cmd);
        hardware_interface::JointHandle frontRightJointHandle(_jointStateInterface.getHandle(_frontRightMotorJointInfo.first),
                                                             &_frontRightMotorJointInfo.second.cmd);

        _velocityJointInterface.registerHandle(rearLeftJointHandle);
        _velocityJointInterface.registerHandle(rearRightJointHandle);
        _velocityJointInterface.registerHandle(frontLeftJointHandle);
        _velocityJointInterface.registerHandle(frontRightJointHandle);



        _havePanTilt = false;
        ros::param::param<bool>("have_pan_tilt", _havePanTilt, true);
        if (_havePanTilt)
        {

            //////////////////////////////////////////
            //
           //        _jointStateInterface.registerHandle(rearLeftJointStateHandle);
//
            //hardware_interface::JointHandle rearLeftJointHandle(_jointStateInterface.getHandle(_rearLeftMotorJointInfo.first),
            //&_rearLeftMotorJointInfo.second.cmd);
            //       _velocityJointInterface.registerHandle(rearLeftJointHandle);

            //////////////////////////////////////////
            _panInfo = joint_pair(panJointName, JointInfo_t());
            _tiltInfo = joint_pair(tiltJointName, JointInfo_t());

            _dynamixelProController = new dynamixel_pro_controller::DynamixelProController(&_jointStateInterface, &_posJointInterface);
            _dynamixelProController->startBroadcastingJointStates();

            hardware_interface::JointStateHandle panJointStateHandle(_panInfo.first,
                                                                     &_panInfo.second.position,
                                                                     &_panInfo.second.velocity,
                                                                     &_panInfo.second.effort );

            hardware_interface::JointStateHandle tiltJointStateHandle(_tiltInfo.first,
                                                                      &_tiltInfo.second.position,
                                                                      &_tiltInfo.second.velocity,
                                                                      &_tiltInfo.second.effort );

        }
    }

    LiziRobot::~LiziRobot() {
        if (_havePanTilt)
            delete _dynamixelProController;
        _dynamixelProController = NULL;
    }

    void LiziRobot::registerInterfaces() {
        RobotBase::registerInterfaces();
    }

    void LiziRobot::read() {
        RobotBase::read();
        if(_havePanTilt)
            _dynamixelProController->read();
    }

    void LiziRobot::write() {
        RobotBase::write();

        if(_havePanTilt)
            _dynamixelProController->write();
    }
}