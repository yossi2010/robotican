//
// Created by tom on 30/10/16.
//

#ifndef ROBOTICAN_HARDWARE_INTERFACE_DYNAMIXEL_CONTROLLER_H
#define ROBOTICAN_HARDWARE_INTERFACE_DYNAMIXEL_CONTROLLER_H

#include <cmath>
#include <string>
#include <sstream>
#include <fstream>
#include <stdlib.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <XmlRpcValue.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/posvel_command_interface.h>
#include <robotican_hardware_interface/dynamixel_driver.h>

#include "yaml-cpp/yaml.h"
#ifdef HAVE_NEW_YAMLCPP
// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
    i = node.as<T>();
}

#endif



namespace dynamixel_controller {

    struct JointInfo_t {
        double position;
        double effort;
        double velocity;
        double cmd_pos;
        double cmd_vel;
        JointInfo_t() {
            position = effort = velocity = cmd_pos = 0;
            cmd_vel = 0;
        }
    };

    struct dynamixel_spec
    {
        std::string name;
        uint16_t model_number;
        int cpr;
        double gear_reduction;
    };

    struct dynamixel_info
    {
        int id;
        std::string joint_name;
        std::string model_name;
        uint16_t model_number;
        uint32_t model_info;
        int cpr;
        double gear_reduction;
        float protocolVer;
    };

    enum control_mode
    {
        POSITION_CONTROL = 3,
        VELOCITY_CONTROL = 1,
        TORQUE_CONTROL = 0,
        UNKOWN  = -1
    };

    struct dynamixel_status
    {
        int id;
        control_mode mode;
        bool torque_enabled;
    };


    class DynamixelController {
    private:
        ros::NodeHandle _nodeHandle;
        hardware_interface::JointStateInterface* _jointStateInterface;
        hardware_interface::PosVelJointInterface* _posVelJointInterface;
        hardware_interface::PositionJointInterface* _positionJointInterface;
        std::map<uint16_t, dynamixel_spec> _modelSpec;
        std::map<std::string, dynamixel_info> _joint2Dxl;
        std::map<int, dynamixel_status> _id2status;
        std::map<std::string, JointInfo_t> _jointsInfo;
        dynamixel_driver::DynamixelDriver* _driver;
        bool _first;
        double _initSpeed;

        void initSpecFile();

        void initPort();

        bool initMotors();

        bool torqueMotors();

        int32_t posToTicks(double rads, const dynamixel_info &info);

        double posToRads(int32_t ticks, const dynamixel_info &info);

        int32_t getDriverVelocity(const dynamixel_info &info, const double velocity) const;

        double getVelocity(const dynamixel_info &info, int32_t velocity) const;

    public:

        DynamixelController(hardware_interface::JointStateInterface* jointStateInterface,
                            hardware_interface::PosVelJointInterface* posVelJointInterface,
                            hardware_interface::PositionJointInterface* positionJointInterface);

        ~DynamixelController();

        void read();

        void write();
    };
}

#endif //ROBOTICAN_HARDWARE_INTERFACE_DYNAMIXEL_CONTROLLER_H
