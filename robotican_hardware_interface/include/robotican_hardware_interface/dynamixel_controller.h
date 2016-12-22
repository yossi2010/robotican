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
#include <sensor_msgs/JointState.h>
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
    /*!
     * @brief Struct that use to represent the current joint info info
     */
    struct JointInfo_t {
        double position;
        double effort;
        double velocity;
        double cmd_pos;
        double cmd_vel;
        double pre_vel;

        JointInfo_t() {
            position = effort = velocity = cmd_pos = 0;
            cmd_vel = pre_vel = 0;

        }
    };
    /*!
     * @brief Struct to represent the dxl motor specification
     */
    struct dynamixel_spec
    {
        std::string name;
        uint16_t model_number;
        int cpr;
        double gear_reduction;
    };
    /*!
     * @brief Struct to represent the dxl motor info
     */
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
        int torque;
        bool readPos;
        bool readVel;
        bool readEff;
        bool writePos;
        bool writeVel;
        bool useMinVel;
        bool velocityMode;
        dynamixel_info() {
            writePos = writeVel  = readPos = readVel = readEff = useMinVel = true;
            velocityMode = false;
        }
    };

    enum control_mode
    {
        POSITION_CONTROL = 3,
        VELOCITY_CONTROL = 1,
        TORQUE_CONTROL = 0,
        UNKOWN  = -1
    };
    /*!
     * @brief Struct to represent the current dxl motor status
     */
    struct dynamixel_status
    {
        int id;
        control_mode mode;
        bool torque_enabled;
    };

    /*!
     * @brief Class to control a set to dxl motors, this class will manage the communication to dxl driver.
     */
    class DynamixelController {
    private:
        ros::NodeHandle _nodeHandle;
        hardware_interface::JointStateInterface* _jointStateInterface;
        hardware_interface::PosVelJointInterface* _posVelJointInterface;
        hardware_interface::PositionJointInterface* _positionJointInterface;
        std::map<uint16_t, dynamixel_spec> _modelSpec;                                                                  //!< Map That contain the motors specification
        std::map<std::string, dynamixel_info> _joint2Dxl;                                                               //!< Map that contain the current dxl info.
        std::map<int, dynamixel_status> _id2status;
        std::map<std::string, JointInfo_t> _jointsInfo;                                                                 //!< Map that contain the current joint info
        dynamixel_driver::DynamixelDriver* _driver;                                                                     //!< Communication channel.

        ros::Publisher _jointStatePub;                                                                                  //!< Publisher for the current joint state.

        ros::Subscriber _torqueListener;
        ros::Subscriber _cmdListener;                                                                                   //!< Trigger which fire when sending a msg to topic 'joint_command'

        bool _first;                                                                                                    //!< Indicate if got the first read
        double _initSpeedProtocol1;                                                                                     //!< The stating up speed when the motor (protocol 1.0)
        double _initSpeedProtocol2;                                                                                     //!< The stating up speed when the motor (protocol 2.0)

        /*!
         * @brief Read from dxl motor specification file and init the '_modelSpec' field
         */
        void initSpecFile();
        /*!
         * @brief Get parameters and Build the '_driver' field, (the transport layer).
         */
        void initPort();
        /*!
         * @brief Build all the dxl motors.
         * @return If succeed or not.
         */
        bool initMotors();
        /*!
         * @brief Method that enable/disable torque for all the dxl motors.
         * @return If succeed or not.
         */
        bool torqueMotors();
        /*!
         * @brief Method that convert rad in the current joint to ticks.
         * @param rads: the current joint rad.
         * @param info: the current dxl info.
         * @return The ticks
         */
        int32_t posToTicks(double rads, const dynamixel_info &info);
        /*!
         * @brief Method that convert ticks to rads.
         * @param ticks: The current motor ticks
         * @param info: The current dxl info
         * @return The rads
         */
        double posToRads(int32_t ticks, const dynamixel_info &info);
        /*!
         * @brief Method that get the current velocity to send to the driver.
         * @param info: The current dxl motor info.
         * @param velocity: The current velocity to be converted.
         * @return Driver velocity.
         */
        int32_t getDriverVelocity(const dynamixel_info &info, const double velocity) const;
        /*!
         * @brief Method that get the driver velocity the convert it to joint velocity.
         * @param info: The current dxl motor info
         * @param velocity: The current driver velocity to be converted
         * @return Joint velocity
         */
        double getVelocity(const dynamixel_info &info, int32_t velocity) const;

        bool testBit(int16_t number, int16_t offset);
        /*!
         * @brief Register all the dxl joints in a hardware interfaces
         */
        void registerJointHandlers();
        /*!
         * @brief Build all the dxl motor joints
         */
        void buildJoints();
        /*!
         * @brief Trigger which fire when a node send msgs to 'joint_command' topic
         * @param msg: The current cmd of all the joints
         */
        void CmdCallback(const sensor_msgs::JointStateConstPtr &msg);

    public:

        DynamixelController();

        DynamixelController(hardware_interface::JointStateInterface* jointStateInterface,
                            hardware_interface::PosVelJointInterface* posVelJointInterface,
                            hardware_interface::PositionJointInterface* positionJointInterface);

        ~DynamixelController();
        /*!
         * Do the reading for all the dxl motor.
         */
        void read();
        /*!
         * Publish the current state of the dxl motors.
         */
        void publishState();
        /*!
         * Write the current command.
         */
        void write();
    };
}

#endif //ROBOTICAN_HARDWARE_INTERFACE_DYNAMIXEL_CONTROLLER_H
