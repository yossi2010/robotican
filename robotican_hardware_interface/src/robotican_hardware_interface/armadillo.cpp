//
// Created by tom on 07/04/16.
//

#include "robotican_hardware_interface/armadillo.h"

//#define DEBUG_ARMADILLO

namespace robotican_hardware {
    ArmadilloRobot::ArmadilloRobot() : RobotBase(),
                                       _positionJointInterface(),_posVelJointInterface() {


        std::string  leftFingerPubTopic, leftFingerSubTopic,
                leftFingerJointName,
                rightFingerPubTopic, rightFingerSubTopic, rightFingerJointName, panPubTopic, panSubTopic, panJoint,tiltPubTopic, tiltSubTopic, tiltJoint;
        _dynamixelProController = NULL;
        _boardManager.buildDevices(&_jointStateInterface, &_positionJointInterface);
        if(!_nodeHandle.getParam("left_finger_topic_pub", leftFingerPubTopic) ||
           !_nodeHandle.getParam("left_finger_topic_sub", leftFingerSubTopic) ||
           !_nodeHandle.getParam("left_finger_joint", leftFingerJointName) ||
           !_nodeHandle.getParam("right_finger_topic_pub", rightFingerPubTopic) ||
           !_nodeHandle.getParam("right_finger_topic_sub", rightFingerSubTopic) ||
           !_nodeHandle.getParam("right_finger_joint", rightFingerJointName) ||
           !_nodeHandle.getParam("pan_topic_pub", panPubTopic) ||
           !_nodeHandle.getParam("pan_topic_sub", panSubTopic) ||
           !_nodeHandle.getParam("pan_joint", panJoint) ||
           !_nodeHandle.getParam("tilt_topic_pub", tiltPubTopic) ||
           !_nodeHandle.getParam("tilt_topic_sub", tiltSubTopic) ||
           !_nodeHandle.getParam("tilt_joint", tiltJoint)) {                        /* parameters that must be instalize for the robot to work*/
            ros::shutdown();
        }
        else {
            bool haveArm = true;
            ros::param::param<bool>("hava_arm", haveArm, true);
            if(haveArm) {
                _dynamixelProController = new dynamixel_pro_controller::DynamixelProController(&_jointStateInterface, &_posVelJointInterface);
                _dynamixelProController->startBroadcastingJointStates();
            }
            _first[0] = _first[1] = false;
            _leftFingerCmd = _nodeHandle.advertise<std_msgs::Float64>(leftFingerPubTopic, 10);
            _rightFingerCmd = _nodeHandle.advertise<std_msgs::Float64>(rightFingerPubTopic, 10);

            _panCmd = _nodeHandle.advertise<std_msgs::Float64>(panPubTopic, 10);
            _tiltCmd = _nodeHandle.advertise<std_msgs::Float64>(tiltPubTopic, 10);

            _leftFingerState = _nodeHandle.subscribe<dynamixel_msgs::JointState>(leftFingerSubTopic, 10,
                                                                                 &ArmadilloRobot::leftFingerCallback,
                                                                                 this);
            _rightFingerState = _nodeHandle.subscribe<dynamixel_msgs::JointState>(rightFingerSubTopic, 10,
                                                                                  &ArmadilloRobot::rightFingerCallback,
                                                                                  this);

            _panState = _nodeHandle.subscribe<dynamixel_msgs::JointState>(panSubTopic, 10, &ArmadilloRobot::panCallback,
                                                                          this);
            _tiltState = _nodeHandle.subscribe<dynamixel_msgs::JointState>(tiltSubTopic, 10,
                                                                           &ArmadilloRobot::tiltCallback, this);


            _leftFingerInfo = std::pair<std::string, JointInfo_t>(leftFingerJointName, JointInfo_t());
            _rightFingerInfo = std::pair<std::string, JointInfo_t>(rightFingerJointName, JointInfo_t());

            _panInfo = std::pair<std::string, JointInfo_t>(panJoint, JointInfo_t());
            _tiltInfo = std::pair<std::string, JointInfo_t>(tiltJoint, JointInfo_t());

            hardware_interface::JointStateHandle leftJointStateHandle(_leftFingerInfo.first,
                                                                      &_leftFingerInfo.second.position,
                                                                      &_leftFingerInfo.second.velocity,
                                                                      &_leftFingerInfo.second.effort);

            hardware_interface::JointStateHandle rightJointStateHandle(_rightFingerInfo.first,
                                                                       &_rightFingerInfo.second.position,
                                                                       &_rightFingerInfo.second.velocity,
                                                                       &_rightFingerInfo.second.effort);

            hardware_interface::JointStateHandle panJointStateHandle(_panInfo.first,
                                                                     &_panInfo.second.position,
                                                                     &_panInfo.second.velocity,
                                                                     &_panInfo.second.effort);

            hardware_interface::JointStateHandle tiltJointStateHandle(_tiltInfo.first,
                                                                      &_tiltInfo.second.position,
                                                                      &_tiltInfo.second.velocity,
                                                                      &_tiltInfo.second.effort);

            _jointStateInterface.registerHandle(leftJointStateHandle);
            _jointStateInterface.registerHandle(rightJointStateHandle);

            _jointStateInterface.registerHandle(panJointStateHandle);
            _jointStateInterface.registerHandle(tiltJointStateHandle);

            hardware_interface::JointHandle leftJointHandle(_jointStateInterface.getHandle(_leftFingerInfo.first),
                                                            &_leftFingerInfo.second.cmd);
            hardware_interface::JointHandle rightJointHandle(_jointStateInterface.getHandle(_rightFingerInfo.first),
                                                             &_rightFingerInfo.second.cmd);

            hardware_interface::JointHandle panJointHandle(_jointStateInterface.getHandle(_panInfo.first),
                                                           &_panInfo.second.cmd);
            hardware_interface::JointHandle tiltJointHandle(_jointStateInterface.getHandle(_tiltInfo.first),
                                                            &_tiltInfo.second.cmd);

            _positionJointInterface.registerHandle(leftJointHandle);
            _positionJointInterface.registerHandle(rightJointHandle);

            _positionJointInterface.registerHandle(panJointHandle);
            _positionJointInterface.registerHandle(tiltJointHandle);
        }

    }



    void ArmadilloRobot::panCallback(const dynamixel_msgs::JointState::ConstPtr &msg) {
        _panInfo.second.position = msg->current_pos;
        _panInfo.second.velocity = msg->velocity;
        _panInfo.second.effort = msg->load;
    }

    void ArmadilloRobot::tiltCallback(const dynamixel_msgs::JointState::ConstPtr &msg) {
        _tiltInfo.second.position = msg->current_pos;
        _tiltInfo.second.velocity = msg->velocity;
        _tiltInfo.second.effort = msg->load;

    }

    void ArmadilloRobot::leftFingerCallback(const dynamixel_msgs::JointState::ConstPtr &msg) {

        _leftFingerInfo.second.position = msg->current_pos;
        _leftFingerInfo.second.velocity = msg->velocity;
        _leftFingerInfo.second.effort = msg->load;
        if (!_first[0]) {

            _leftFingerInfo.second.cmd = _leftFingerInfo.second.position;
            _first[0]=true;
        }

    }

    void ArmadilloRobot::rightFingerCallback(const dynamixel_msgs::JointState::ConstPtr &msg) {
        _rightFingerInfo.second.position = msg->current_pos;
        _rightFingerInfo.second.velocity = msg->velocity;
        _rightFingerInfo.second.effort = msg->load;
        if (!_first[1]) {

            _rightFingerInfo.second.cmd = _rightFingerInfo.second.position;
            _first[1]=true;
        }

    }

    void ArmadilloRobot::registerInterfaces() {
        RobotBase::registerInterfaces();
        registerInterface(&_positionJointInterface);
        registerInterface(&_posVelJointInterface);
    }

    void ArmadilloRobot::read() {
        RobotBase::read();
        if(_dynamixelProController != NULL)
            _dynamixelProController->read();
    }

    void ArmadilloRobot::write() {
        RobotBase::write();
        if(_dynamixelProController != NULL)
            _dynamixelProController->write();
        std_msgs::Float64 leftMsg, rightMsg, panMsg, tiltMsg;

        panMsg.data = _panInfo.second.cmd;
        tiltMsg.data = _tiltInfo.second.cmd;
        _panCmd.publish(panMsg);
        _tiltCmd.publish(tiltMsg);

        if (_first[0] && _first[1]) {
            leftMsg.data = _leftFingerInfo.second.cmd;
            rightMsg.data = _rightFingerInfo.second.cmd;


            _leftFingerCmd.publish(leftMsg);
            _rightFingerCmd.publish(rightMsg);

        }

    }

    ArmadilloRobot::~ArmadilloRobot() {
        if(_dynamixelProController != NULL) {
            delete _dynamixelProController;
            _dynamixelProController = NULL;
        }

    }
}

