//
// Created by tom on 27/10/16.
//

#ifndef ROBOTICAN_HARDWARE_INTERFACE_DYNAMIXEL_DRIVER_H
#define ROBOTICAN_HARDWARE_INTERFACE_DYNAMIXEL_DRIVER_H

#include <ros/ros.h>
#include <dynamixel_sdk/dynamixel_sdk.h>


// Control table address FOR MX-28
#define ADDR_MX_MODEL_NUM               0
#define ADDR_MX_TORQUE_ENABLE           24                  // Control table address is different in Dynamixel model
#define ADDR_MX_GOAL_POSITION           30
#define ADDR_MX_GOAL_SPEED              32
#define ADDR_MX_PRESENT_POSITION        36
#define ADDR_MX_PRESENT_SPEED           38
#define ADDR_MX_PRESENT_LOAD            40
#define ADDR_MX_PRESENT_TEMPERATURE     43
#define ADDR_MX_MOVING                  46
#define ADDR_MX_GOAL_ACCELERATION       73

// Control table address FOR Pro
#define ADDR_PRO_MODEL_NUM               0
#define ADDR_PRO_TORQUE_ENABLE           562                  // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION           596
#define ADDR_PRO_GOAL_SPEED              600
#define ADDR_PRO_GOAL_ACCELERATION       606
#define ADDR_PRO_PRESENT_POSITION        611
#define ADDR_PRO_PRESENT_SPEED           615
#define ADDR_PRO_PRESENT_CURRENT         621
#define ADDR_PRO_PRESENT_TEMPERATURE     43
#define ADDR_PRO_MOVING                  46

// Protocol version
#define PROTOCOL1_VERSION                1.0                 // See which protocol version is used in the Dynamixel
#define PROTOCOL2_VERSION                2.0                 // See which protocol version is used in the Dynamixel

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque

namespace dynamixel_driver {

    struct DxlMotorInfo_t {
        uint8_t id;
        float protocol;
    };

    enum DriverMode {
        COMBINE = 0,
        PROTOCOL1 = 1,
        PROTOCOL2 = 2
    };

    class DynamixelDriver {
    private:
        dynamixel::PortHandler *_portHandler;
        dynamixel::PacketHandler *_packetHandlerVer1;
        dynamixel::PacketHandler *_packetHandlerVer2;

        bool pingMotorProtocol1(uint8_t id);

        bool pingMotorProtocol2(uint8_t id);

        bool setMotorTorqueProtocol1(uint8_t id, uint8_t torque = TORQUE_ENABLE);

        bool setMotorTorqueProtocol2(uint8_t id, uint8_t torque = TORQUE_ENABLE);

        bool setMotorPositionProtocol1(uint8_t id, uint16_t ticks);

        bool setMotorPositionProtocol2(uint8_t id, uint32_t ticks);

        bool setMotorSpeedProtocol1(uint8_t id, uint32_t speed);

        bool setMotorSpeedProtocol2(uint8_t id, uint32_t speed);

        bool setMotorAccelerationProtocol1(uint8_t id, uint32_t acceleration);

        bool setMotorAccelerationProtocol2(uint8_t id, uint32_t acceleration);

        bool getMotorPositionProtocol1(uint8_t id, uint32_t &position);

        bool getMotorPositionProtocol2(uint8_t id, uint32_t &position);

        bool getMotorSpeedProtocol1(uint8_t id, uint32_t &speed);

        bool getMotorSpeedProtocol2(uint8_t id, uint32_t &speed);

        bool getMotorLoadProtocol1(uint8_t id, uint16_t &load);

        bool getMotorLoadProtocol2(uint8_t id, uint16_t &load);

        bool getMotorModelProtocol1(uint8_t id, uint16_t &model);

        bool getMotorModelProtocol2(uint8_t id, uint16_t &model);

    public:
        DynamixelDriver(const char *port, unsigned int baudrate, DriverMode mode);

        ~DynamixelDriver();

        bool isPortOpen();

        bool pingMotor(DxlMotorInfo_t motorInfo);

        bool setMotorTorque(DxlMotorInfo_t motorInfo, uint8_t torque);

        bool setMotorPosition(DxlMotorInfo_t motorInfo, uint32_t ticks);

        bool setMotorSpeed(DxlMotorInfo_t motorInfo, uint32_t speed);

        bool setMotorAcceletarion(DxlMotorInfo_t motorInfo, uint32_t acceleration);

        bool getMotorPosition(DxlMotorInfo_t motorInfo, uint32_t &position);

        bool getMotorSpeed(DxlMotorInfo_t motorInfo, uint32_t &speed);

        bool getMotorLoad(DxlMotorInfo_t motorInfo, uint16_t &load);

        bool getMotorModel(DxlMotorInfo_t motorInfo, uint16_t &model);
    };
}

#endif //ROBOTICAN_HARDWARE_INTERFACE_DYNAMIXEL_DRIVER_H
