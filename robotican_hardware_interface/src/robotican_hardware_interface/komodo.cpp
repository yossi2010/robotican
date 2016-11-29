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
            std::vector<std::string> armJoints, gripperJoints;
            ros::param::param("arm_joints", armJoints, std::vector<std::string>());
            ros::param::param("gripper_joints", gripperJoints, std::vector<std::string>());

            for(std::vector<std::string>::iterator jointName = armJoints.begin(); jointName != armJoints.end(); ++jointName) {
                _jointInfo.insert(std::pair<std::string, dynamixel_controller::JointInfo_t>((*jointName), dynamixel_controller::JointInfo_t()));
                dynamixel_controller::JointInfo_t &info = _jointInfo[(*jointName)];
                hardware_interface::JointStateHandle jointStateHandle((*jointName), &info.position, &info.velocity ,&info.effort);
                _jointStateInterface.registerHandle(jointStateHandle);
                hardware_interface::PosVelJointHandle posVelJointHandle(_jointStateInterface.getHandle((*jointName)),  &info.cmd_pos, &info.cmd_vel);
                _posVelJointInterface.registerHandle(posVelJointHandle);

            }

            for(std::vector<std::string>::iterator jointName = gripperJoints.begin(); jointName != gripperJoints.end(); ++jointName) {
                _jointInfo.insert(std::pair<std::string, dynamixel_controller::JointInfo_t>((*jointName), dynamixel_controller::JointInfo_t()));
                dynamixel_controller::JointInfo_t &info = _jointInfo[(*jointName)];
                hardware_interface::JointStateHandle jointStateHandle((*jointName), &info.position, &info.velocity ,&info.effort);
                _jointStateInterface.registerHandle(jointStateHandle);
                hardware_interface::JointHandle jointHandle(_jointStateInterface.getHandle((*jointName)),  &info.cmd_pos);
                _positionJointInterface.registerHandle(jointHandle);

            }
            _armStateListener = _nodeHandle.subscribe("dxl_joint_states", 10, &KomodoRobot::armStateCallback, this);
            _armCmd = _nodeHandle.advertise<sensor_msgs::JointState>("joint_command", 10);
            //_dynamixelController = new dynamixel_controller::DynamixelController(&_jointStateInterface, &_posVelJointInterface, &_positionJointInterface);
        }

    }

    KomodoRobot::~KomodoRobot() {
//        if(_dynamixelController != NULL) {
//            delete _dynamixelController;
//            _dynamixelController = NULL;
//        }
    }

    void KomodoRobot::registerInterfaces() {
        RobotBase::registerInterfaces();
        registerInterface(&_positionJointInterface);
        registerInterface(&_posVelJointInterface);

    }

    void KomodoRobot::read() {
        RobotBase::read();
//        if(_dynamixelController != NULL)
//            _dynamixelController->read();
    }

    void KomodoRobot::write() {
        RobotBase::write();
        sensor_msgs::JointState jointCmd;
        for(std::map<std::string, dynamixel_controller::JointInfo_t>::iterator it = _jointInfo.begin(); it != _jointInfo.end(); ++it) {
            std::string jointName = it->first;
            dynamixel_controller::JointInfo_t info = it->second;

            jointCmd.name.push_back(jointName);
            jointCmd.position.push_back(info.cmd_pos);
            jointCmd.velocity.push_back(info.cmd_vel);
        }
        _armCmd.publish(jointCmd);
//        if(_dynamixelController != NULL)
//            _dynamixelController->write();


    }

    void KomodoRobot::armStateCallback(const sensor_msgs::JointStateConstPtr &msg) {
        size_t size = msg->name.size();
        for(int i = 0; i < size; ++i) {
            std::string jointName = msg->name[i];
            dynamixel_controller::JointInfo_t &jointInfo = _jointInfo[jointName];
            jointInfo.position = msg->position[i];
            jointInfo.effort = msg->effort[i];
            jointInfo.velocity = msg->velocity[i];
        }
    }


}
