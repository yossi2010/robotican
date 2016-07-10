//
// Created by tom on 08/05/16.
//

#ifndef ROBOTICAN_HARDWARE_INTERFACE_RICBOARDMANAGER_H
#define ROBOTICAN_HARDWARE_INTERFACE_RICBOARDMANAGER_H

#include <vector>
#include <ros/ros.h>
#include <ros/package.h>
#include <boost/thread/thread.hpp>
#include <robotican_hardware_interface/Device.h>
#include <robotican_hardware_interface/Battery.h>
#include <robotican_hardware_interface/Servo.h>
#include <robotican_hardware_interface/ros_utils.h>
#include <hardware_interface/joint_command_interface.h>
#include <robotican_hardware_interface/TransportLayer.h>
#include <robotican_hardware_interface/Ultrasonic.h>
#include <robotican_hardware_interface/Gps.h>
#include <robotican_hardware_interface/Imu.h>
#include <robotican_hardware_interface/RiCMotor.h>
#include <robotican_hardware_interface/Switch.h>
#include <robotican_hardware_interface/Relay.h>

#include <map>
#include <dynamic_reconfigure/server.h>
#include <robotican_hardware_interface/RiCBoardManager.h>
#include <robotican_hardware_interface/RiCBoardConfig.h>
#include <robotican_hardware_interface/RiCBoardServoConfig.h>



#define MAX_BUFF_SIZE 255
#define PC_VERSION 100
#define RIC_BOARD_DEBUG

namespace robotican_hardware {
    class CloseLoopMotor;
    class CloseLoopMotorWithPotentiometer;

    class ServoParamHandler {
    private:
        ros::NodeHandle _nodeHandle;
        std::map<std::string, Servo*> _servos;
        dynamic_reconfigure::Server <robotican_hardware_interface::RiCBoardServoConfig> _server;
        dynamic_reconfigure::Server<robotican_hardware_interface::RiCBoardServoConfig>::CallbackType _callbackType;

        void dynamicCallback(robotican_hardware_interface::RiCBoardServoConfig &config, uint32_t level);

        Servo* checkIfJointValid(std::string jointName);

    public:
        ServoParamHandler();

        void add(std::string jointName, Servo* servo);

        void remove(std::string jointName);

    };



    class RiCBoardManager {
    private:
        ServoParamHandler _servoParamHandler;

        byte _rcvBuff[MAX_BUFF_SIZE];
        TransportLayer _transportLayer;
        ConnectEnum::ConnectEnum  _connectState;
        ros::NodeHandle _nodeHandle;
        ros::Timer _sendKeepAliveTimer;
        ros::Timer _timeoutKeepAliveTimer;
        ros::AsyncSpinner _spinner;
        std::vector<Device*> _devices;
        byte _idGen;

        unsigned int getBaudrate();

        std::string getPort();

        void resetBuff();

        void setConnectState(ConnectEnum::ConnectEnum connectState);

        void debugMsgHandler(DebugMsg *debugMsg);

        void clear();

    public:
        RiCBoardManager();

        void buildDevices();

        void buildDevices(hardware_interface::JointStateInterface*, hardware_interface::VelocityJointInterface*);

        void buildDevices(hardware_interface::JointStateInterface*, hardware_interface::PositionJointInterface*);

        void connect();

        void disconnect();

        void handleMessage();

        void connectionHandle(ConnectState *connectState);

        void sendKeepAliveEvent(const ros::TimerEvent &timerEvent);

        void timeoutKeepAliveEvent(const ros::TimerEvent &timerEvent);

        void keepAliveHandle(KeepAliveMsg *keepAliveMsg);

        void deviceMessageHandler(DeviceMessage *deviceMsg);

        void write();

        ConnectEnum::ConnectEnum getConnectState();
    };
}

#endif //ROBOTICAN_HARDWARE_INTERFACE_RICBOARDMANAGER_H
