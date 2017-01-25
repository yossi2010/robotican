//
// Created by tom on 14/04/16.
//

#include "robotican_hardware_interface/LiziRobot.h"

namespace robotican_hardware {

    LiziRobot::LiziRobot() : RobotBase()
    {
        //_havePanTilt = false;
        //ros::param::param<bool>("have_pan_tilt", _havePanTilt, true);

//        if (_havePanTilt)
//        {
//            preparePanTilt();
//            buildConnections();
//        }
    } //delsdfld

    LiziRobot::~LiziRobot() {

    }

    void LiziRobot::registerInterfaces() {
        RobotBase::registerInterfaces();
    }

    void LiziRobot::read() {
        RobotBase::read();
    }

    void LiziRobot::write() {
        RobotBase::write();
//        std_msgs::Float64 panMsg, tiltMsg;
//
//        panMsg.data = _panInfo.second.cmd;
//        tiltMsg.data = _tiltInfo.second.cmd;
//        _panPub.publish(panMsg);
//        _tiltPub.publish(tiltMsg);
    }

//    void LiziRobot::preparePanTilt()
//    {
//        std::string panJointName,
//                    tiltJointName;
//
//        _nodeHandle.getParam("pan_joint", panJointName);
//        _nodeHandle.getParam("tilt_joint", tiltJointName);
//
//        _panInfo = joint_pair(panJointName, JointInfo_t());
//        _tiltInfo = joint_pair(tiltJointName, JointInfo_t());
//
//        hardware_interface::JointStateHandle panJointStateHandle(_panInfo.first,
//                                                                 &_panInfo.second.position,
//                                                                 &_panInfo.second.velocity,
//                                                                 &_panInfo.second.effort );
//
//        hardware_interface::JointStateHandle tiltJointStateHandle(_tiltInfo.first,
//                                                                  &_tiltInfo.second.position,
//                                                                  &_tiltInfo.second.velocity,
//                                                                  &_tiltInfo.second.effort );
//
//        _jointStateInterface.registerHandle(panJointStateHandle);
//        _jointStateInterface.registerHandle(tiltJointStateHandle);
//
//        hardware_interface::JointHandle panJointHandle(_jointStateInterface.getHandle(_panInfo.first),
//                                                       &_panInfo.second.cmd);
//        hardware_interface::JointHandle tiltJointHandle(_jointStateInterface.getHandle(_tiltInfo.first),
//                                                        &_tiltInfo.second.cmd);
//
//        _posJointInterface.registerHandle(panJointHandle);
//        _posJointInterface.registerHandle(tiltJointHandle);
//    }

//    void LiziRobot::buildConnections()
//    {
//        _nodeHandle.getParam("pan_topic_sub", _panSubTopic);
//        _nodeHandle.getParam("tilt_topic_sub", _tiltSubTopic);
//        _nodeHandle.getParam("pan_topic_pub", _panPubTopic);
//        _nodeHandle.getParam("tilt_topic_pub", _tiltPubTopic);
//        _panPub = _nodeHandle.advertise<std_msgs::Float64>(_panPubTopic, 10);
//        _tiltPub = _nodeHandle.advertise<std_msgs::Float64>(_tiltPubTopic, 10);
//        _panSub = _nodeHandle.subscribe<dynamixel_msgs::JointState>(_panSubTopic, 10, &LiziRobot::panCallBack, this);
//        _tiltSub = _nodeHandle.subscribe<dynamixel_msgs::JointState>(_tiltSubTopic, 10, &LiziRobot::tiltCallBack, this);
//    }

//    void LiziRobot::panCallBack(const dynamixel_msgs::JointState::ConstPtr &msg)
//    {
//        _panInfo.second.position = msg->current_pos;
//        _panInfo.second.velocity = msg->velocity;
//        _panInfo.second.effort = msg->load;
//    }
//
//    void LiziRobot::tiltCallBack(const dynamixel_msgs::JointState::ConstPtr &msg)
//    {
//        _tiltInfo.second.position = msg->current_pos;
//        _tiltInfo.second.velocity = msg->velocity;
//        _tiltInfo.second.effort = msg->load;
//    }
    }
