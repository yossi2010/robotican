//
// Created by tom on 30/10/16.
//

#include "robotican_hardware_interface/dynamixel_controller.h"

namespace dynamixel_controller {
    void DynamixelController::initSpecFile() {
        std::string path = ros::package::getPath("robotican_hardware_interface");
        path += "/config/motor_data.yaml";
        YAML::Node doc;

#ifdef HAVE_NEW_YAMLCPP
        doc = YAML::LoadFile(path);
#else
        ifstream fin(path.c_str());
        YAML::Parser parser(fin);
        parser.GetNextDocument(doc);
#endif
        for (int i = 0; i < doc.size(); i++)
        {
            dynamixel_spec spec;

            // Load the basic specs of this motor type
            doc[i]["name"] >> spec.name;
            doc[i]["model_number"] >> spec.model_number;
            doc[i]["cpr"]  >> spec.cpr;
            doc[i]["gear_reduction"]  >> spec.gear_reduction;

            _modelSpec[spec.model_number] = spec;
        }

    }

    void DynamixelController::initPort() {
        std::string device;
        int baudrate;
        int timeout_ms;

        _nodeHandle.param<std::string>("device", device, "/dev/ttyUSB0");
        _nodeHandle.param<int>("baudrate_t", baudrate, 57600);

        _driver = new dynamixel_driver::DynamixelDriver(device.c_str(), (unsigned int) baudrate, dynamixel_driver::COMBINE);
        if(_driver == NULL) ROS_BREAK();
        if(!_driver->isPortOpen()) {
            ROS_ERROR("[%s]: port %s can't be open", ros::this_node::getName().c_str(), device.c_str());
            ros::shutdown();
        }
    }

    DynamixelController::~DynamixelController() {
        if(_driver != NULL) {
            delete _driver;
        }
        _driver = NULL;
    }

    DynamixelController::DynamixelController(hardware_interface::JointStateInterface *jointStateInterface,
                                             hardware_interface::PosVelJointInterface *posVelJointInterface,
                                             hardware_interface::PositionJointInterface *positionJointInterface) {
        _posVelJointInterface = posVelJointInterface;
        _positionJointInterface = positionJointInterface;
        _jointStateInterface = jointStateInterface;
        _driver = NULL;
        _first = true;
        initSpecFile();
        initPort();
        if(ros::ok()) {
            if(initMotors()) {
//                torqueMotors();
            } else {
                ROS_ERROR("[%s]: failed to construct motor, please restart the robot and try again.", ros::this_node::getName().c_str());
                ros::shutdown();
            }

        }
    }

    bool DynamixelController::initMotors() {
        int num_motors = 0;
        if(_nodeHandle.hasParam("servos")) {
            XmlRpc::XmlRpcValue servos;
            _nodeHandle.getParam("servos", servos);
            if(!servos.getType() == XmlRpc::XmlRpcValue::TypeArray) {
                ROS_ERROR("[%s]: Invalid/missing servo information on the param server", ros::this_node::getName().c_str());
                return false;
            } else {
                num_motors = servos.size();
                for(int i = 0; i < num_motors && ros::ok(); ++i) {
                    dynamixel_info info;
                    if(!servos[i].getType() == XmlRpc::XmlRpcValue::TypeStruct) {
                        ROS_ERROR("[%s]: Invalid/Missing info-struct for servo index %d",ros::this_node::getName().c_str() ,i);
                        return false;
                    } else {

                        if(!servos[i]["id"].getType() == XmlRpc::XmlRpcValue::TypeInt) {
                            ROS_ERROR("[%s]: Invalid/Missing id for servo index %d", ros::this_node::getName().c_str(), i);
                            return false;
                        } else {
                            info.id = static_cast<int>(servos[i]["id"]);
                        }
                        if (!servos[i]["joint_name"].getType() == XmlRpc::XmlRpcValue::TypeString)
                        {
                            ROS_ERROR("[%s]: Invalid/Missing joint name for servo index %d, id: %d", ros::this_node::getName().c_str() ,i, info.id);
                            return false;
                        } else {
                            info.joint_name = static_cast<std::string>(servos[i]["joint_name"]);
                        }

                        if(!servos[i]["protocol_version"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
                            ROS_ERROR("[%s]: Invalid/Missing protocol version for servo index %d, id: %d", ros::this_node::getName().c_str() ,i, info.id);
                            return false;
                        } else {
                            dynamixel_driver::DxlMotorInfo_t dxlMotorInfo;
                            dxlMotorInfo.id = (uint8_t) info.id;
                            dxlMotorInfo.protocol = (float) static_cast<double>(servos[i]["protocol_version"]);
                            info.protocolVer = dxlMotorInfo.protocol;
                            if(_driver->pingMotor(dxlMotorInfo)) {
                                bool success = true;
                                success &= _driver->getMotorModel(dxlMotorInfo, (uint16_t &) info.model_number);
                                success &= _modelSpec.find(info.model_number) != _modelSpec.end();
                                if(success) {
                                    info.cpr = _modelSpec[info.model_number].cpr;
                                    info.gear_reduction = _modelSpec[info.model_number].gear_reduction;
                                    _joint2Dxl[info.joint_name] = info;

                                    dynamixel_status status;
                                    status.id = info.id;
                                    status.mode = UNKOWN;
                                    status.torque_enabled = false;
                                    _id2status[info.id] = status;



                                }
                                else {
                                    ROS_ERROR("[%s]: Failed to load model information for dynamixel id %d", ros::this_node::getName().c_str(),info.id);
                                    return false;
                                }
                            } else {
                                ROS_ERROR("[%s]: Cannot ping dyanmixel id: %d", ros::this_node::getName().c_str(),info.id);
                                return false;
                            }
                        }
                    }
                }
                for(std::map<std::string, dynamixel_info>::iterator it=_joint2Dxl.begin(); it != _joint2Dxl.end(); ++it) {
                    std::string jointName = it->first;
                    _jointsInfo.insert(std::pair<std::string, JointInfo_t>(jointName, JointInfo_t()));


                    hardware_interface::JointStateHandle jointStateHandle(jointName, &_jointsInfo[jointName].position, &_jointsInfo[jointName].velocity,&_jointsInfo[jointName].effort);
                    _jointStateInterface->registerHandle(jointStateHandle);
                    if(_joint2Dxl[jointName].protocolVer == PROTOCOL2_VERSION) {
                        hardware_interface::PosVelJointHandle jointHandle(_jointStateInterface->getHandle(jointName)
                                , &_jointsInfo[jointName].cmd_pos, &_jointsInfo[jointName].cmd_vel);
                        _posVelJointInterface->registerHandle(jointHandle);
                    } else {
                        hardware_interface::JointHandle jointHandle(_jointStateInterface->getHandle(jointName) ,
                                                                    &_jointsInfo[jointName].cmd_pos);
                        _positionJointInterface->registerHandle(jointHandle);
                    }

                }
            }
        } else {
            ROS_ERROR("[%s]: No servos details loaded to param server", ros::this_node::getName().c_str());
            return false;
        }

        return true;
    }

    uint32_t DynamixelController::posToTicks(double rads, const dynamixel_info &info) {
        double cprDev2 = info.cpr / 2.0f;

        return static_cast<uint32_t>(round((rads * cprDev2 / M_PI) + cprDev2));
    }

    double DynamixelController::posToRads(uint32_t ticks, const dynamixel_info &info) {
        double cprDev2 = info.cpr / 2.0f;
        ROS_INFO("%u %f %d %f", ticks, cprDev2, info.id, ((double)ticks - cprDev2) * M_PI / cprDev2);
        return ((double)ticks - cprDev2) * M_PI / cprDev2;
    }

    bool DynamixelController::torqueMotors() {
        for(std::map<std::string, dynamixel_info>::iterator it=_joint2Dxl.begin(); it != _joint2Dxl.end(); ++it) {
            dynamixel_driver::DxlMotorInfo_t info;
            info.id = (uint8_t) it->second.id;
            info.protocol = it->second.protocolVer;
            if(!_driver->setMotorTorque(info, TORQUE_ENABLE)) return false;
        }
        return true;
    }

    void DynamixelController::read() {
        for (std::map<std::string, dynamixel_info>::iterator iter = _joint2Dxl.begin(); iter != _joint2Dxl.end(); iter++) {
            std::string jointName = iter->first;
            dynamixel_info info = iter->second;
            uint32_t position = 0, velocity = 0 ;
            uint16_t load = 0 ;
            dynamixel_driver::DxlMotorInfo_t dxlMotorInfo;
            dxlMotorInfo.id = info.id;
            dxlMotorInfo.protocol = info.protocolVer;


            if(_driver->getMotorPosition(dxlMotorInfo, position)) {
                double rad = posToRads(position, info);
                _jointsInfo[jointName].position = rad;
                if (_first) _jointsInfo[jointName].cmd_pos = rad;

            }
            if(_driver->getMotorSpeed(dxlMotorInfo, velocity)) {
                double radVel = getVelocity(info, velocity);
                _jointsInfo[jointName].velocity = radVel;
            }

            if(_driver->getMotorLoad(dxlMotorInfo, load)) {
                double effort = load;
                _jointsInfo[jointName].effort = effort;
            }
            ros::Duration(3.0).sleep();
        }
        if(_first) _first = false;

    }

    double DynamixelController::getVelocity(const dynamixel_info &info, uint32_t velocity) const {
        return ((double) velocity) * 2.0 * M_PI / 60.0 / info.gear_reduction;
    }

    void DynamixelController::write() {
//        for (std::map<std::string, dynamixel_info>::iterator iter = _joint2Dxl.begin(); iter != _joint2Dxl.end(); iter++) {
//            std::string jointName = iter->first;
//            dynamixel_info info = iter->second;
//            JointInfo_t jointInfo = _jointsInfo[jointName];
//            dynamixel_driver::DxlMotorInfo_t dxlMotorInfo;
//            dxlMotorInfo.id = info.id;
//            dxlMotorInfo.protocol = info.protocolVer;
//
//            uint32_t ticks = posToTicks(jointInfo.cmd_pos, info);
//            uint32_t speed = getDriverVelocity(info, jointInfo.cmd_vel);
//
//            if(!_driver->setMotorSpeed(dxlMotorInfo, speed)) {
//                ROS_WARN("[%s]: Unable to set speed", ros::this_node::getName().c_str());
//            }
//
//            if(!_driver->setMotorPosition(dxlMotorInfo, ticks)) {
//                ROS_WARN("[%s]: Unable to set position", ros::this_node::getName().c_str());
//            }
//
//
//        }
    }

    uint32_t DynamixelController::getDriverVelocity(const dynamixel_info &info, const double velocity) const { return static_cast<uint32_t >(velocity / 2.0 / M_PI * 60.0 * info.gear_reduction); }


}