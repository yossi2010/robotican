//
// Created by tom on 27/10/16.
//

#include "robotican_hardware_interface/dynamixel_driver.h"
namespace dynamixel_driver {
    DynamixelDriver::DynamixelDriver(const char *port, unsigned int baudrate, DriverMode mode) {
        _portHandler = dynamixel::PortHandler::getPortHandler(port);

        if(isPortOpen()) {
            _portHandler->setBaudRate(baudrate);
            switch (mode) {
                case COMBINE:
                    _packetHandlerVer1 = dynamixel::PacketHandler::getPacketHandler(PROTOCOL1);
                    _packetHandlerVer2 = dynamixel::PacketHandler::getPacketHandler(PROTOCOL2);
                    break;
                case PROTOCOL1:
                    _packetHandlerVer1 = dynamixel::PacketHandler::getPacketHandler(PROTOCOL1);
                    _packetHandlerVer2 = NULL;

                    break;
                case PROTOCOL2:
                    _packetHandlerVer2 = dynamixel::PacketHandler::getPacketHandler(PROTOCOL2);
                    _packetHandlerVer1 = NULL;
                    break;
            }

        }
        else ROS_INFO("[%s]: Port %s can't be open", ros::this_node::getName().c_str(), port);

    }

    DynamixelDriver::~DynamixelDriver() {
        if (_portHandler != NULL) {
            _portHandler->closePort();
            delete _portHandler;
        }

        if(_packetHandlerVer1 != NULL) {
            delete _packetHandlerVer1;
        }

        if(_packetHandlerVer2 != NULL) {
            delete _packetHandlerVer2;
        }

        _portHandler = NULL;
        _packetHandlerVer1 = _packetHandlerVer2 = NULL;
    }

    bool DynamixelDriver::pingMotorProtocol1(uint8_t id) {
        int dxl_comm_result = COMM_TX_FAIL;
        uint8_t dxl_error;

        dxl_comm_result = _packetHandlerVer1->ping(_portHandler, id, &dxl_error);
        if(dxl_comm_result != COMM_SUCCESS) _packetHandlerVer1->printTxRxResult(dxl_comm_result);
        else if(dxl_error != 0) _packetHandlerVer1->printRxPacketError(dxl_error);
        else return true;

        return false;
    }

    bool DynamixelDriver::pingMotorProtocol2(uint8_t id) {
        return false;
    }

    bool DynamixelDriver::pingMotor(DxlMotorInfo_t motorInfo) {
        bool gotPing = false;

        if(motorInfo.protocol == PROTOCOL1_VERSION) gotPing = pingMotorProtocol1(motorInfo.id);
        else if(motorInfo.protocol == PROTOCOL2_VERSION) gotPing = pingMotorProtocol2(motorInfo.id);

        return gotPing;
    }

    bool DynamixelDriver::setMotorTorqueProtocol1(uint8_t id, uint8_t torque) {
        int dxl_comm_result = COMM_TX_FAIL;
        uint8_t dxl_error;

        dxl_comm_result = _packetHandlerVer1->write1ByteTxRx(_portHandler, id, ADDR_MX_TORQUE_ENABLE, torque, &dxl_error);
        if(dxl_comm_result != COMM_SUCCESS) _packetHandlerVer1->printTxRxResult(dxl_comm_result);
        else if(dxl_error != 0) _packetHandlerVer1->printRxPacketError(dxl_error);
        else return true;

        return false;
    }

    bool DynamixelDriver::isPortOpen() {
        return _portHandler->openPort();
    }

    bool DynamixelDriver::setMotorTorque(DxlMotorInfo_t motorInfo, uint8_t torque) {

        bool setTorque = false;

        if(motorInfo.protocol == PROTOCOL1_VERSION) setTorque = setMotorTorqueProtocol1(motorInfo.id, torque);
        else if(motorInfo.protocol == PROTOCOL2_VERSION) setTorque = setMotorTorqueProtocol2(motorInfo.id, torque);

        return setTorque;
    }

    bool DynamixelDriver::setMotorTorqueProtocol2(uint8_t id, uint8_t torque) {
        return false;
    }

    bool DynamixelDriver::setMotorPositionProtocol1(uint8_t id, uint16_t ticks) {
        int dxl_comm_result = COMM_TX_FAIL;
        uint8_t dxl_error;

        dxl_comm_result = _packetHandlerVer1->write2ByteTxRx(_portHandler, id, ADDR_MX_GOAL_POSITION, ticks, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) _packetHandlerVer1->printTxRxResult(dxl_comm_result);
        else if (dxl_error != 0) _packetHandlerVer1->printRxPacketError(dxl_error);
        else return true;

        return false;
    }

    bool DynamixelDriver::setMotorPositionProtocol2(uint8_t id, int ticks) {
        return false;
    }

    bool DynamixelDriver::setMotorPosition(DxlMotorInfo_t motorInfo, uint16_t ticks) {
        bool sendPositionCmd = false;

        if(motorInfo.protocol == PROTOCOL1_VERSION) sendPositionCmd = setMotorPositionProtocol1(motorInfo.id, ticks);
        else if(motorInfo.protocol == PROTOCOL2_VERSION) sendPositionCmd = setMotorPositionProtocol1(motorInfo.id, ticks);

        return sendPositionCmd;
    }
}
