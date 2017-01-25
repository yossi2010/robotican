//
// Created by tom on 07/04/16.
//

#include "robotican_hardware_interface/armadillo.h"

//#define DEBUG_ARMADILLO

namespace robotican_hardware {
    ArmadilloRobot::ArmadilloRobot() : RobotBase(),
                                       _positionJointInterface(),_posVelJointInterface() {

        _first = false;
        std::string panPubTopic, panSubTopic, panJoint,tiltPubTopic, tiltSubTopic, tiltJoint;
        _boardManager.buildDevices(&_jointStateInterface, &_positionJointInterface);
        if(!_nodeHandle.getParam("pan_topic_pub", panPubTopic) ||
           !_nodeHandle.getParam("pan_topic_sub", panSubTopic) ||
           !_nodeHandle.getParam("pan_joint", panJoint) ||
           !_nodeHandle.getParam("tilt_topic_pub", tiltPubTopic) ||
           !_nodeHandle.getParam("tilt_topic_sub", tiltSubTopic) ||
           !_nodeHandle.getParam("tilt_joint", tiltJoint)) {                        /* parameters that must be instalize for the robot to work*/
            ros::shutdown();
        }
        else {
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
                _armStateListener = _nodeHandle.subscribe("dxl_joint_states", 10, &ArmadilloRobot::armStateCallback, this);
                _armCmd = _nodeHandle.advertise<sensor_msgs::JointState>("joint_command", 10);
            }

            _panCmd = _nodeHandle.advertise<std_msgs::Float64>(panPubTopic, 10);
            _tiltCmd = _nodeHandle.advertise<std_msgs::Float64>(tiltPubTopic, 10);


            _panState = _nodeHandle.subscribe<dynamixel_msgs::JointState>(panSubTopic, 10, &ArmadilloRobot::panCallback,
                                                                          this);
            _tiltState = _nodeHandle.subscribe<dynamixel_msgs::JointState>(tiltSubTopic, 10,
                                                                           &ArmadilloRobot::tiltCallback, this);



            _panInfo = std::pair<std::string, JointInfo_t>(panJoint, JointInfo_t());
            _tiltInfo = std::pair<std::string, JointInfo_t>(tiltJoint, JointInfo_t());


            hardware_interface::JointStateHandle panJointStateHandle(_panInfo.first,
                                                                     &_panInfo.second.position,
                                                                     &_panInfo.second.velocity,
                                                                     &_panInfo.second.effort);

            hardware_interface::JointStateHandle tiltJointStateHandle(_tiltInfo.first,
                                                                      &_tiltInfo.second.position,
                                                                      &_tiltInfo.second.velocity,
                                                                      &_tiltInfo.second.effort);


            _jointStateInterface.registerHandle(panJointStateHandle);
            _jointStateInterface.registerHandle(tiltJointStateHandle);



            hardware_interface::JointHandle panJointHandle(_jointStateInterface.getHandle(_panInfo.first),
                                                           &_panInfo.second.cmd);
            hardware_interface::JointHandle tiltJointHandle(_jointStateInterface.getHandle(_tiltInfo.first),
                                                            &_tiltInfo.second.cmd);

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





    void ArmadilloRobot::registerInterfaces() {
        RobotBase::registerInterfaces();
        registerInterface(&_positionJointInterface);
        registerInterface(&_posVelJointInterface);
    }

    void ArmadilloRobot::read() {
        RobotBase::read();

    }

    void ArmadilloRobot::write() {
        RobotBase::write();

        std_msgs::Float64  panMsg, tiltMsg;

        panMsg.data = _panInfo.second.cmd;
        tiltMsg.data = _tiltInfo.second.cmd;
        _panCmd.publish(panMsg);
        _tiltCmd.publish(tiltMsg);

        if(_first) {
            sensor_msgs::JointState jointCmd;
            for (std::map<std::string, dynamixel_controller::JointInfo_t>::iterator it = _jointInfo.begin();
                 it != _jointInfo.end(); ++it) {
                std::string jointName = it->first;
                dynamixel_controller::JointInfo_t info = it->second;

                jointCmd.name.push_back(jointName);
                jointCmd.position.push_back(info.cmd_pos);
                jointCmd.velocity.push_back(info.cmd_vel);
            }
            _armCmd.publish(jointCmd);
        }


    }

    ArmadilloRobot::~ArmadilloRobot() {


    }

    void ArmadilloRobot::armStateCallback(const sensor_msgs::JointStateConstPtr &msg) {

        size_t size = msg->name.size();
        for(int i = 0; i < size; ++i) {
            std::string jointName = msg->name[i];
            dynamixel_controller::JointInfo_t &jointInfo = _jointInfo[jointName];
            jointInfo.position = msg->position[i];
            jointInfo.effort = msg->effort[i];
            jointInfo.velocity = msg->velocity[i];
//ROS_INFO("joint: %s,  jointInfo.cmd_vel=%f\n",jointName.c_str(),jointInfo.velocity);
            if(!_first) {

               // jointInfo.cmd_vel = msg->velocity[i];
                jointInfo.cmd_pos = msg->position[i];
            }
        }

        _first = true;
    }
}

