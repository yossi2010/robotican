//
// Created by tom on 27/10/16.
//

#include "robotican_hardware_interface/dynamixel_driver.h"
namespace dynamixel_driver {
    DynamixelDriver::DynamixelDriver(const char *port, unsigned int baudrate, DriverMode mode) {
        _portHandler = dynamixel::PortHandler::getPortHandler(port);

        if(isPortOpen()) {
            _portHandler->setBaudRate(baudrate);
            _portHandler->setPacketTimeout(10000.0);
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

    bool DynamixelDriver::isPortOpen() {
        return _portHandler->openPort();
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
        int dxl_comm_result = COMM_TX_FAIL;
        uint8_t dxl_error;

        dxl_comm_result = _packetHandlerVer2->ping(_portHandler, id, &dxl_error);
        if(dxl_comm_result != COMM_SUCCESS) _packetHandlerVer2->printTxRxResult(dxl_comm_result);
        else if(dxl_error != 0) _packetHandlerVer2->printRxPacketError(dxl_error);
        else return true;

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

    bool DynamixelDriver::setMotorTorqueProtocol2(uint8_t id, uint8_t torque) {
        int dxl_comm_result = COMM_TX_FAIL;
        uint8_t dxl_error;

        dxl_comm_result = _packetHandlerVer2->write1ByteTxRx(_portHandler, id, ADDR_PRO_TORQUE_ENABLE, torque, &dxl_error);
        if(dxl_comm_result != COMM_SUCCESS) _packetHandlerVer2->printTxRxResult(dxl_comm_result);
        else if(dxl_error != 0) _packetHandlerVer2->printRxPacketError(dxl_error);
        else return true;

        return false;
    }

    bool DynamixelDriver::setMotorTorque(DxlMotorInfo_t motorInfo, uint8_t torque) {

        bool setTorque = false;

        if(motorInfo.protocol == PROTOCOL1_VERSION) setTorque = setMotorTorqueProtocol1(motorInfo.id, torque);
        else if(motorInfo.protocol == PROTOCOL2_VERSION) setTorque = setMotorTorqueProtocol2(motorInfo.id, torque);

        return setTorque;
    }

    bool DynamixelDriver::setMotorPositionProtocol1(uint8_t id, int32_t ticks) {
        int dxl_comm_result = COMM_TX_FAIL;
        uint8_t dxl_error;

        dxl_comm_result = _packetHandlerVer1->write2ByteTxRx(_portHandler, id, ADDR_MX_GOAL_POSITION, (uint16_t) ticks, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) _packetHandlerVer1->printTxRxResult(dxl_comm_result);
        else if (dxl_error != 0) _packetHandlerVer1->printRxPacketError(dxl_error);
        else return true;

        return false;
    }

    bool DynamixelDriver::setMotorPositionProtocol2(uint8_t id, int32_t ticks) {
        int dxl_comm_result = COMM_TX_FAIL;
        uint8_t dxl_error;

        dxl_comm_result = _packetHandlerVer2->write4ByteTxRx(_portHandler, id, ADDR_PRO_GOAL_POSITION, (uint32_t) ticks, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) _packetHandlerVer2->printTxRxResult(dxl_comm_result);
        else if (dxl_error != 0) _packetHandlerVer2->printRxPacketError(dxl_error);
        else return true;

        return false;
    }

    bool DynamixelDriver::setMotorPosition(DxlMotorInfo_t motorInfo, int32_t ticks) {
        bool sendPositionCmd = false;

        if(motorInfo.protocol == PROTOCOL1_VERSION) sendPositionCmd = setMotorPositionProtocol1(motorInfo.id, ticks);
        else if(motorInfo.protocol == PROTOCOL2_VERSION) sendPositionCmd = setMotorPositionProtocol2(motorInfo.id, ticks);

        return sendPositionCmd;
    }

    bool DynamixelDriver::setMotorSpeedProtocol1(uint8_t id, int32_t speed) {
        int dxl_comm_result = COMM_TX_FAIL;
        uint8_t dxl_error;

        dxl_comm_result = _packetHandlerVer1->write2ByteTxRx(_portHandler, id, ADDR_MX_GOAL_SPEED, (uint16_t) speed, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) _packetHandlerVer1->printTxRxResult(dxl_comm_result);
        else if (dxl_error != 0) _packetHandlerVer1->printRxPacketError(dxl_error);
        else return true;

        return false;
    }

    bool DynamixelDriver::setMotorSpeedProtocol2(uint8_t id, int32_t speed) {
        int dxl_comm_result = COMM_TX_FAIL;
        uint8_t dxl_error;

        dxl_comm_result = _packetHandlerVer2->write4ByteTxRx(_portHandler, id, ADDR_PRO_GOAL_SPEED, (uint32_t) speed, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) _packetHandlerVer2->printTxRxResult(dxl_comm_result);
        else if (dxl_error != 0) _packetHandlerVer2->printRxPacketError(dxl_error);
        else return true;

        return false;
    }

    bool DynamixelDriver::setMotorSpeed(DxlMotorInfo_t motorInfo, int32_t speed) {
        bool sendSpeedCmd = false;

        if(motorInfo.protocol == PROTOCOL1_VERSION) sendSpeedCmd = setMotorSpeedProtocol1(motorInfo.id, speed);
        else if(motorInfo.protocol == PROTOCOL2_VERSION) sendSpeedCmd = setMotorSpeedProtocol2(motorInfo.id, speed);

        return sendSpeedCmd;
    }

    bool DynamixelDriver::setMotorAccelerationProtocol1(uint8_t id, int32_t acceleration) {
        int dxl_comm_result = COMM_TX_FAIL;
        uint8_t dxl_error;

        dxl_comm_result = _packetHandlerVer1->write1ByteTxRx(_portHandler, id, ADDR_MX_GOAL_ACCELERATION, (uint8_t) acceleration, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) _packetHandlerVer1->printTxRxResult(dxl_comm_result);
        else if (dxl_error != 0) _packetHandlerVer1->printRxPacketError(dxl_error);
        else return true;

        return false;
    }

    bool DynamixelDriver::setMotorAccelerationProtocol2(uint8_t id, int32_t acceleration) {
        int dxl_comm_result = COMM_TX_FAIL;
        uint8_t dxl_error;

        dxl_comm_result = _packetHandlerVer2->write4ByteTxRx(_portHandler, id, ADDR_PRO_GOAL_ACCELERATION, (uint32_t) acceleration, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) _packetHandlerVer2->printTxRxResult(dxl_comm_result);
        else if (dxl_error != 0) _packetHandlerVer2->printRxPacketError(dxl_error);
        else return true;

        return false;
    }

    bool DynamixelDriver::setMotorAcceletarion(DxlMotorInfo_t motorInfo, int32_t acceleration) {
        bool sendSpeedCmd = false;

        if(motorInfo.protocol == PROTOCOL1_VERSION) sendSpeedCmd = setMotorAccelerationProtocol1(motorInfo.id, acceleration);
        else if(motorInfo.protocol == PROTOCOL2_VERSION) sendSpeedCmd = setMotorAccelerationProtocol2(motorInfo.id, acceleration);

        return sendSpeedCmd;
    }

    bool DynamixelDriver::getMotorPositionProtocol1(uint8_t id, int32_t &position) {
        int dxl_comm_result = COMM_TX_FAIL;
        uint8_t dxl_error;
        int16_t read;
        dxl_comm_result = _packetHandlerVer1->read2ByteTxRx(_portHandler, id, ADDR_MX_PRESENT_POSITION, (uint16_t *) &read, &dxl_error);
        position = read;
        if (dxl_comm_result != COMM_SUCCESS) _packetHandlerVer1->printTxRxResult(dxl_comm_result);
        else if (dxl_error != 0) _packetHandlerVer1->printRxPacketError(dxl_error);
        else return true;

        return false;
    }

    bool DynamixelDriver::getMotorPositionProtocol2(uint8_t id, int32_t &position) {
        int dxl_comm_result = COMM_TX_FAIL;
        uint8_t dxl_error;
        int32_t read;
        dxl_comm_result = _packetHandlerVer2->read4ByteTxRx(_portHandler, id, ADDR_PRO_PRESENT_POSITION, (uint32_t *) &read, &dxl_error);
        position = read;
        if (dxl_comm_result != COMM_SUCCESS) _packetHandlerVer2->printTxRxResult(dxl_comm_result);
        else if (dxl_error != 0) _packetHandlerVer2->printRxPacketError(dxl_error);
        else return true;

        return false;
    }

    bool DynamixelDriver::getMotorPosition(DxlMotorInfo_t motorInfo, int32_t &position) {
        bool gotCurrentPos = false;

        if(motorInfo.protocol == PROTOCOL1_VERSION) gotCurrentPos = getMotorPositionProtocol1(motorInfo.id, position);
        else if(motorInfo.protocol == PROTOCOL2_VERSION) gotCurrentPos = getMotorPositionProtocol2(motorInfo.id, position);

        return gotCurrentPos;
    }

    bool DynamixelDriver::getMotorSpeedProtocol1(uint8_t id, int32_t &speed) {
        int dxl_comm_result = COMM_TX_FAIL;
        uint8_t dxl_error;
        int16_t read;
        dxl_comm_result = _packetHandlerVer1->read2ByteTxRx(_portHandler, id, ADDR_MX_PRESENT_SPEED, (uint16_t *) &read, &dxl_error);
        speed = read;
        if (dxl_comm_result != COMM_SUCCESS) _packetHandlerVer1->printTxRxResult(dxl_comm_result);
        else if (dxl_error != 0) _packetHandlerVer1->printRxPacketError(dxl_error);
        else return true;

        return false;
    }

    bool DynamixelDriver::getMotorSpeedProtocol2(uint8_t id, int32_t &speed) {

#ifdef FULL_ARM_PARAMS
        int dxl_comm_result = COMM_TX_FAIL;
        uint8_t dxl_error;
        int32_t read;
        dxl_comm_result = _packetHandlerVer2->read4ByteTxRx(_portHandler, id, ADDR_PRO_PRESENT_SPEED, (uint32_t *) &read, &dxl_error);
        speed = read;
        if (dxl_comm_result != COMM_SUCCESS) _packetHandlerVer2->printTxRxResult(dxl_comm_result);
        else if (dxl_error != 0) _packetHandlerVer2->printRxPacketError(dxl_error);
        else return true;

        return false;
#else
        speed = 0;
        return true;
#endif
    }

    bool DynamixelDriver::getMotorSpeed(DxlMotorInfo_t motorInfo, int32_t &speed) {
        bool gotCurrentSpeed = false;

        if(motorInfo.protocol == PROTOCOL1_VERSION) gotCurrentSpeed = getMotorSpeedProtocol1(motorInfo.id, speed);

        else if(motorInfo.protocol == PROTOCOL2_VERSION) gotCurrentSpeed = getMotorSpeedProtocol2(motorInfo.id, speed);

        return gotCurrentSpeed;
    }

    bool DynamixelDriver::getMotorLoadProtocol1(uint8_t id, int16_t &load) {
        int dxl_comm_result = COMM_TX_FAIL;
        uint8_t dxl_error;
        int16_t read;
        dxl_comm_result = _packetHandlerVer1->read2ByteTxRx(_portHandler, id, ADDR_MX_PRESENT_LOAD, (uint16_t *) &read, &dxl_error);
        load = read;
        if (dxl_comm_result != COMM_SUCCESS) _packetHandlerVer1->printTxRxResult(dxl_comm_result);
        else if (dxl_error != 0) _packetHandlerVer1->printRxPacketError(dxl_error);
        else return true;

        return false;
    }

    bool DynamixelDriver::getMotorLoadProtocol2(uint8_t id, int16_t &load) {
#ifdef FULL_ARM_PARAMS
        int dxl_comm_result = COMM_TX_FAIL;
        uint8_t dxl_error;
        int16_t read;
        dxl_comm_result = _packetHandlerVer2->read2ByteTxRx(_portHandler, id, ADDR_PRO_PRESENT_CURRENT, (uint16_t *) &read, &dxl_error);
        load = read;
        if (dxl_comm_result != COMM_SUCCESS) _packetHandlerVer2->printTxRxResult(dxl_comm_result);
        else if (dxl_error != 0) _packetHandlerVer2->printRxPacketError(dxl_error);
        else return true;

        return false;
#else
        load = 0;
        return true;
#endif
    }

    bool DynamixelDriver::getMotorLoad(DxlMotorInfo_t motorInfo, int16_t &load) {
        bool gotCurrentLoad = false;

        if(motorInfo.protocol == PROTOCOL1_VERSION) gotCurrentLoad = getMotorLoadProtocol1(motorInfo.id, load);
        else if(motorInfo.protocol == PROTOCOL2_VERSION) gotCurrentLoad = getMotorLoadProtocol2(motorInfo.id, load);

        return gotCurrentLoad;
    }

    bool DynamixelDriver::getMotorModelProtocol1(uint8_t id, uint16_t &model) {
        int dxl_comm_result = COMM_TX_FAIL;
        uint8_t dxl_error;

        dxl_comm_result = _packetHandlerVer1->read2ByteTxRx(_portHandler, id, ADDR_MX_MODEL_NUM, &model, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) _packetHandlerVer1->printTxRxResult(dxl_comm_result);
        else if (dxl_error != 0) _packetHandlerVer1->printRxPacketError(dxl_error);
        else return true;

        return false;
    }

    bool DynamixelDriver::getMotorModelProtocol2(uint8_t id, uint16_t &model) {
        int dxl_comm_result = COMM_TX_FAIL;
        uint8_t dxl_error;

        dxl_comm_result = _packetHandlerVer2->read2ByteTxRx(_portHandler, id, ADDR_PRO_MODEL_NUM, &model, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) _packetHandlerVer2->printTxRxResult(dxl_comm_result);
        else if (dxl_error != 0) _packetHandlerVer2->printRxPacketError(dxl_error);
        else return true;

        return false;
    }

    bool DynamixelDriver::getMotorModel(DxlMotorInfo_t motorInfo, uint16_t &model) {
        bool gotModel = false;

        if(motorInfo.protocol == PROTOCOL1_VERSION) gotModel = getMotorModelProtocol1(motorInfo.id, model);
        else if(motorInfo.protocol == PROTOCOL2_VERSION) gotModel = getMotorModelProtocol2(motorInfo.id, model);

        return gotModel;
    }
}
