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



#define MAX_BUFF_SIZE 255                                                   //!< The max buffer size
#define PC_VERSION 100                                                      //!< The current pc version, the RiCBoard also have is define, If they don't mach the program won't run.
#define RIC_BOARD_DEBUG                                                     //!< Use to enable debug mode.

namespace robotican_hardware {
    class CloseLoopMotor;                                                   // To avoid circular dependency
    class CloseLoopMotorWithPotentiometer;                                  // To avoid circular dependency




    //!@brief This is the call that manage the communication beteen the PC and the RiCBoard
    class RiCBoardManager {
    private:
        byte _rcvBuff[MAX_BUFF_SIZE];                                       //!< Receive buffer: To receive the package for the RiCBoard
        TransportLayer _transportLayer;                                     //!< Transport layer manage the sending and receiving packages To and from the RiCBoard
        ConnectEnum::ConnectEnum  _connectState;                            //!< The current connection state
        ros::NodeHandle _nodeHandle;                                        //!< If you don't know what this is please refer to http://wiki.ros.org
        ros::Timer _sendKeepAliveTimer;                                     //!< Trigger that fire every 3/sec and send a keep alive pkg to the RiCBoard
        ros::Timer _timeoutKeepAliveTimer;                                  //!< Trigger that fire if the RiCBoard didn't send keep alive
        ros::AsyncSpinner _spinner;
        std::vector<Device*> _devices;                                      //!< Array that contain all the device that the RiCboard had build
        byte _idGen;                                                        //!< Generator for the device ids

        /*!
         * @return Get the current communication baud rate
         */
        unsigned int getBaudrate();
        /*!
         * @return Get the current port name
         */
        std::string getPort();
        /*!
         * @brief Reset the '_rcvBuff' buffer
         */
        void resetBuff();
        /*!
         * @brief Change the current connection state.
         * @param connectState: The new connection state.
         */
        void setConnectState(ConnectEnum::ConnectEnum connectState);
        /*!
         * @brief Get a debug message from the RiCBoard and print it.
         * @param debugMsg: The incoming debug message from the RiCBoard
         */
        void debugMsgHandler(DebugMsg *debugMsg);
        /*!
         * @brief Erase all device that the manager had build
         */
        void clear();

    public:

        RiCBoardManager();
        /*!
         * @brief Build devices that don't need hardware interface
         */
        void buildDevices();
        /*!
         * @brief Build device that need JointStateInterface and VelocityJointInterface
         */
        void buildDevices(hardware_interface::JointStateInterface*, hardware_interface::VelocityJointInterface*);
        /*!
         * @brief Build device that need JointStateInterface and PositionJointInterface
         */
        void buildDevices(hardware_interface::JointStateInterface*, hardware_interface::PositionJointInterface*);
//        void buildDevices(hardware_interface::JointStateInterface*, hardware_interface::PositionJointInterface*);
        /*!
         * @brief Connect to the RiCBoard
         */
        void connect();
        /*!
         * @brief Disconnect to the RiCBoard
         */
        void disconnect();
        /*!
         * @brief Handle the incoming msgs from the RiCboard
         */
        void handleMessage();
        /*!
         * @brief Handle the connection msgs
         * @param connectState: Incoming connection msg from the RiCBoard
         */
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
