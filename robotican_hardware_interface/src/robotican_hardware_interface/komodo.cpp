//
// Created by tom on 17/04/16.
//
#include "robotican_hardware_interface/komodo.h"

namespace robotican_hardware {


    KomodoRobot::KomodoRobot() {

        _dynamixelController = NULL;


        bool haveArm = true;
        ros::param::param<bool>("have_arm", haveArm, true);
        if(haveArm) {
            //_dynamixelController = new dynamixel_controller::DynamixelController(&_jointStateInterface, &_posVelJointInterface, &_positionJointInterface);
        }

    }

    KomodoRobot::~KomodoRobot() {
        if(_dynamixelController != NULL) {
            delete _dynamixelController;
            _dynamixelController = NULL;
        }
    }

    void KomodoRobot::registerInterfaces() {
        RobotBase::registerInterfaces();
        registerInterface(&_positionJointInterface);
        registerInterface(&_posVelJointInterface);

    }

    void KomodoRobot::read() {
        RobotBase::read();
        if(_dynamixelController != NULL)
            _dynamixelController->read();
    }

    void KomodoRobot::write() {
        RobotBase::write();
        if(_dynamixelController != NULL)
            _dynamixelController->write();


    }


}