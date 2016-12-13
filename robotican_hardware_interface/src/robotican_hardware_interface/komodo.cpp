//
// Created by tom on 17/04/16.
//
#include "robotican_hardware_interface/komodo.h"

namespace robotican_hardware {

    template <typename T> int sgn(T val) {
        return (T(0) < val) - (val < T(0));
    }

    KomodoRobot::KomodoRobot() {

        _dynamixelController = NULL;
        _torsoJoint = NULL;
        _first = _doneHomingUpper = _doneHomingLower = false;
        _torsoDir = UP;
        _lastTorsoRead = 0.0;
        bool haveArm = true;


        ros::param::param<bool>("have_arm", haveArm, true);

        if(haveArm) {
            if(buildDxlMotors()) {
                _armStateListener = _nodeHandle.subscribe("dxl_joint_states", 10, &KomodoRobot::armStateCallback, this);
                _armCmd = _nodeHandle.advertise<sensor_msgs::JointState>("joint_command", 10);
            }
            else {
                ROS_ERROR("[%s]: Dynamixel motor can't be build shutting down the robot...", ros::this_node::getName().c_str());
                ros::shutdown();
            }
        }
        if(!loadHomeSwitch()){
            ros::shutdown();
        } else {
            _homingService = _nodeHandle.advertiseService("homing", &KomodoRobot::onHomingRequest, this);
        }
    }

    KomodoRobot::~KomodoRobot() {

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
        if(_first) {
            sensor_msgs::JointState jointCmd;
            for (std::map<std::string, dynamixel_controller::JointInfo_t>::iterator it = _jointInfo.begin();
                 it != _jointInfo.end(); ++it) {
                std::string jointName = it->first;
                dynamixel_controller::JointInfo_t info = it->second;
                if(jointName != "torso_joint" || jointName == "torso_joint" && (_doneHomingLower || _doneHomingUpper)) {
                    jointCmd.name.push_back(jointName);
                    jointCmd.position.push_back(info.cmd_pos);
                    jointCmd.velocity.push_back(info.cmd_vel);
                }
            }
            _armCmd.publish(jointCmd);
        }
//        if(_dynamixelController != NULL)
//            _dynamixelController->write();


    }

    void KomodoRobot::armStateCallback(const sensor_msgs::JointStateConstPtr &msg) {
        size_t size = msg->name.size();
        for(int i = 0; i < size; ++i) {

            std::string jointName = msg->name[i];
            dynamixel_controller::JointInfo_t &jointInfo = _jointInfo[jointName];
            if(jointName != "torso_joint") {
                jointInfo.position = msg->position[i];
                jointInfo.effort = msg->effort[i];
                jointInfo.velocity = msg->velocity[i];
                if (!_first) {
                    jointInfo.cmd_vel = msg->velocity[i];
                    jointInfo.cmd_pos = msg->position[i];
                }
            }
            else {
                double currentPosition = msg->position[i];
                if (_doneHomingLower || _doneHomingUpper) {
                    double lastPosition = _lastTorsoRead,  positionDelta = (currentPosition - lastPosition);
                    if (fabs(positionDelta) > 2.24) {
                        positionDelta = -sgn(positionDelta) *  M_PI + positionDelta;
                    }

                    jointInfo.position += positionDelta;
                }
                _lastTorsoRead = currentPosition;
            }
        }

        _first = true;
    }

    bool KomodoRobot::buildDxlMotors() {
        if(_nodeHandle.hasParam("servos")) {
            XmlRpc::XmlRpcValue servos;
            _nodeHandle.getParam("servos", servos);
            if(!servos.getType() == XmlRpc::XmlRpcValue::TypeArray) {
                ROS_ERROR("[%s]: Invalid/missing servo information on the param server",
                          ros::this_node::getName().c_str());
                return false;
            }
            else {
                int motorSize = 0;
                motorSize = servos.size();
                for(int i = 0; i < motorSize; ++i) {
                    std::string jointName, jointInterface;
                    if(!servos[i].getType() == XmlRpc::XmlRpcValue::TypeStruct) {
                        ROS_ERROR("[%s]: Invalid/Missing info-struct for servo index %d",ros::this_node::getName().c_str() ,i);
                        return false;
                    } else {
                        if (!servos[i]["joint_name"].getType() == XmlRpc::XmlRpcValue::TypeString)
                        {
                            ROS_ERROR("[%s]: Invalid/Missing joint name for servo index %d", ros::this_node::getName().c_str() ,i);
                            return false;
                        } else {
                            jointName = static_cast<std::string>(servos[i]["joint_name"]);
                        }

                        if (!servos[i]["interface"].getType() == XmlRpc::XmlRpcValue::TypeString)
                        {
                            ROS_ERROR("[%s]: Invalid/Missing interface for servo index %d", ros::this_node::getName().c_str() ,i);
                            return false;
                        } else {
                            jointInterface = static_cast<std::string>(servos[i]["interface"]);
                        }
                        if(!dxlPut2MapAndRegisterInterface(jointName, jointInterface)) {
                            ROS_ERROR("[%s]: Unknown interface %s", ros::this_node::getName().c_str(), jointInterface.c_str());
                            return false;
                        }
                    }
                }
            }

        } else {
            ROS_ERROR("[%s]: Parameter server down have servos param", ros::this_node::getName().c_str());
            return  false;
        }
        return true;
    }

    bool KomodoRobot::dxlPut2MapAndRegisterInterface(const std::string &jointName, const std::string &jointInterface)  {
        _jointInfo.insert(std::pair<std::string, dynamixel_controller::JointInfo_t>(jointName, dynamixel_controller::JointInfo_t()));
        dynamixel_controller::JointInfo_t &info = _jointInfo[jointName];
        hardware_interface::JointStateHandle jointStateHandle(jointName, &info.position, &info.velocity , &info.effort);
        _jointStateInterface.registerHandle(jointStateHandle);

        if(jointInterface == "posVelInterface") {
            hardware_interface::PosVelJointHandle posVelJointHandle(
                    _jointStateInterface.getHandle(jointName), &info.cmd_pos, &info.cmd_vel);
            _posVelJointInterface.registerHandle(posVelJointHandle);
        } else if(jointInterface == "positionInterface") {
            hardware_interface::JointHandle jointHandle(_jointStateInterface.getHandle(jointName), &info.cmd_pos);
            _positionJointInterface.registerHandle(jointHandle);
        } else if(jointInterface == "velocityInterface") {
            hardware_interface::JointHandle jointHandle(_jointStateInterface.getHandle(jointName), &info.cmd_vel);
            _velocityJointInterface.registerHandle(jointHandle);
            if(jointName == "torso_joint") {
                _torsoJoint = &_jointInfo[jointName];
            }
        } else return false;

        return true;
    }

    bool KomodoRobot::loadHomeSwitch() {
        std::string upperSwitchTopic = "", lowerSwitchTopic = "";
        if(_nodeHandle.getParam("upper_switch_topic", upperSwitchTopic)) {
            _upperSwitchListener = _nodeHandle.subscribe(upperSwitchTopic, 10, &KomodoRobot::onUpperSwitchClick, this);
        } else {
            ROS_ERROR("[%s]: 'upper_switch_topic' is undefined", ros::this_node::getName().c_str());
            return false;
        }

        if(_nodeHandle.getParam("lower_switch_topic", lowerSwitchTopic)) {
            _lowerSwitchListener = _nodeHandle.subscribe(lowerSwitchTopic, 10, &KomodoRobot::onLowerSwitchClick, this);
        } else {
            ROS_ERROR("[%s]: 'upper_switch_topic' is undefined", ros::this_node::getName().c_str());
            return false;
        }
        return true;
    }

    void KomodoRobot::onUpperSwitchClick(const std_msgs::BoolConstPtr &value) {
        if(value->data) {
            if(!_doneHomingUpper) {
                _doneHomingUpper = true;
                _torsoJoint->cmd_pos = 1.2;
                _torsoJoint->cmd_vel = 0.0;
                ROS_INFO("[%s]: Torso reach upper switch homing, calculated to upper position", ros::this_node::getName().c_str());
            }
        }
    }

    void KomodoRobot::onLowerSwitchClick(const std_msgs::BoolConstPtr &value) {
        if(value->data) {
            if(!_doneHomingUpper) {
                _doneHomingLower = true;
                _torsoJoint->cmd_pos = 0.0;
                _torsoJoint->cmd_vel = 0.0;
                ROS_INFO("[%s]: Torso reach lower switch homing, calculated to lower position", ros::this_node::getName().c_str());
            }
        }
    }

    bool KomodoRobot::onHomingRequest(robotican_hardware_interface::TorsoHommingRequest &req,
                                      robotican_hardware_interface::TorsoHommingResponse &res) {
        if(req.homeDir == robotican_hardware_interface::TorsoHommingRequest::HOME_UP) {
            _torsoJoint->cmd_vel = 0.2;
        }
        else if(req.homeDir == robotican_hardware_interface::TorsoHommingRequest::HOME_DOWN) {
            _torsoJoint->cmd_vel = -0.2;
        }
        return true;
    }
}
