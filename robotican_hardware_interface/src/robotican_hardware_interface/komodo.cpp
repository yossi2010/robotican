//
// Created by tom on 17/04/16.
//
#include "robotican_hardware_interface/komodo.h"

namespace robotican_hardware {


    KomodoRobot::KomodoRobot() {
        std::string  leftFingerPubTopic, leftFingerSubTopic, leftFingerJointName,
                rightFingerPubTopic, rightFingerSubTopic, rightFingerJointName;

        _dynamixelProController = NULL;
        if(!_nodeHandle.getParam("left_finger_topic_pub", leftFingerPubTopic) ||
           !_nodeHandle.getParam("left_finger_topic_sub", leftFingerSubTopic) ||
           !_nodeHandle.getParam("left_finger_joint", leftFingerJointName) ||
           !_nodeHandle.getParam("right_finger_topic_pub", rightFingerPubTopic) ||
           !_nodeHandle.getParam("right_finger_topic_sub", rightFingerSubTopic) ||
           !_nodeHandle.getParam("right_finger_joint", rightFingerJointName)) {
            /* parameters that must be instalize for the robot to work*/
            ros::shutdown();
        }
        else {
            bool haveArm = true;
            ros::param::param<bool>("have_arm", haveArm, true);
            if(haveArm) {
                _dynamixelProController = new dynamixel_pro_controller::DynamixelProController(&_jointStateInterface, &_posVelJointInterface);
                _dynamixelProController->startBroadcastingJointStates();
            }
            _first[0] = _first[1] = false;
            _leftFingerCmd = _nodeHandle.advertise<std_msgs::Float64>(leftFingerPubTopic, 10);
            _rightFingerCmd = _nodeHandle.advertise<std_msgs::Float64>(rightFingerPubTopic, 10);

            _leftFingerState = _nodeHandle.subscribe<dynamixel_msgs::JointState>(leftFingerSubTopic, 10,
                                                                                 &KomodoRobot::leftFingerCallback, this);
            _rightFingerState = _nodeHandle.subscribe<dynamixel_msgs::JointState>(rightFingerSubTopic, 10,
                                                                                  &KomodoRobot::rightFingerCallback, this);
            _leftFingerInfo = std::pair<std::string, JointInfo_t>(leftFingerJointName, JointInfo_t());
            _rightFingerInfo = std::pair<std::string, JointInfo_t>(rightFingerJointName, JointInfo_t());

            hardware_interface::JointStateHandle leftJointStateHandle(_leftFingerInfo.first,
                                                                      &_leftFingerInfo.second.position,
                                                                      &_leftFingerInfo.second.velocity,
                                                                      &_leftFingerInfo.second.effort);

            hardware_interface::JointStateHandle rightJointStateHandle(_rightFingerInfo.first,
                                                                       &_rightFingerInfo.second.position,
                                                                       &_rightFingerInfo.second.velocity,
                                                                       &_rightFingerInfo.second.effort);
            _jointStateInterface.registerHandle(leftJointStateHandle);
            _jointStateInterface.registerHandle(rightJointStateHandle);

            hardware_interface::JointHandle leftJointHandle(_jointStateInterface.getHandle(_leftFingerInfo.first),
                                                            &_leftFingerInfo.second.cmd);
            hardware_interface::JointHandle rightJointHandle(_jointStateInterface.getHandle(_rightFingerInfo.first),
                                                             &_rightFingerInfo.second.cmd);

            _positionJointInterface.registerHandle(leftJointHandle);
            _positionJointInterface.registerHandle(rightJointHandle);
        }

    }

    KomodoRobot::~KomodoRobot() {
        if(_dynamixelProController != NULL) {
            delete _dynamixelProController;
            _dynamixelProController = NULL;
        }
    }

    void KomodoRobot::registerInterfaces() {
        RobotBase::registerInterfaces();
    }

    void KomodoRobot::read() {
        RobotBase::read();
    }

    void KomodoRobot::write() {
        RobotBase::write();
        if(_dynamixelProController != NULL)
            _dynamixelProController->write();
        std_msgs::Float64 leftMsg, rightMsg;
        if (_first[0] && _first[1]) {
            leftMsg.data = _leftFingerInfo.second.cmd;
            rightMsg.data = _rightFingerInfo.second.cmd;


            _leftFingerCmd.publish(leftMsg);
            _rightFingerCmd.publish(rightMsg);
        }


    }

    void KomodoRobot::leftFingerCallback(const dynamixel_msgs::JointState::ConstPtr &msg) {
        _leftFingerInfo.second.position = msg->current_pos;
        _leftFingerInfo.second.velocity = msg->velocity;
        _leftFingerInfo.second.effort = msg->load;
        if (!_first[0]) {

            _leftFingerInfo.second.cmd = _leftFingerInfo.second.position;
            _first[0]=true;
        }
    }

    void KomodoRobot::rightFingerCallback(const dynamixel_msgs::JointState::ConstPtr &msg) {
        _rightFingerInfo.second.position = msg->current_pos;
        _rightFingerInfo.second.velocity = msg->velocity;
        _rightFingerInfo.second.effort = msg->load;
        if (!_first[1]) {

            _rightFingerInfo.second.cmd = _rightFingerInfo.second.position;
            _first[1]=true;
        }
    }
}