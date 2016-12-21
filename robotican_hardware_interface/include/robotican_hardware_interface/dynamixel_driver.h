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
        dynamixel::PortHandler *_portHandler;                                                               //!< As the name implies, object that handle the transport layer.
        dynamixel::PacketHandler *_packetHandlerVer1;                                                       //!< Helper that warp the package build business logic. (protocol 1.0)
        dynamixel::PacketHandler *_packetHandlerVer2;                                                       //!< Helper that warp the package build business logic. (protocol 2.0)
        /*!
         * @brief Protocol 1.0
         * @param id: Motor to be ping
         * @return If the motor is present or not
         */
        bool pingMotorProtocol1(uint8_t id);
        /*!
         * @brief Protocol 2.0
         * @param id: Motor to be ping
         * @return If the motor is present or not
         */
        bool pingMotorProtocol2(uint8_t id);
        /*!
         * @brief Protocol 1.0
         * @param id: Motor to be torque enable or disable
         * @param torque: To enable or disable motor torque
         * @return If succeed to enable torque or disable torque
         */
        bool setMotorTorqueProtocol1(uint8_t id, uint8_t torque = TORQUE_ENABLE);
        /*!
         * @brief Protocol 2.0
         * @param id: Motor to be torque enable or disable
         * @param torque: To enable or disable motor torque
         * @return If succeed to enable torque or disable torque
         */

        bool setMotorTorqueProtocol2(uint8_t id, uint8_t torque = TORQUE_ENABLE);
        /*!
         * @brief Protocol 1.0
         * @param id: The motor id
         * @param ticks: The position in ticks
         * @return If succeed to send the position command or not
         */
        bool setMotorPositionProtocol1(uint8_t id, int32_t ticks);
        /*!
         * @brief Protocol 2.0
         * @param id: The motor id
         * @param ticks: The position in ticks
         * @return If succeed to send the position command or not
         */
        bool setMotorPositionProtocol2(uint8_t id, int32_t ticks);
        /*!
         * @brief Protocol 1.0
         * @param id: The motor id
         * @param speed: The motor goal speed
         * @return If succeed to send the speed command or not
         */

        bool setMotorSpeedProtocol1(uint8_t id, int32_t speed);
        /*!
         * @brief Protocol 2.0
         * @param id: The motor id
         * @param speed: The motor goal speed
         * @return If succeed to send the speed command or not
         */
        bool setMotorSpeedProtocol2(uint8_t id, int32_t speed);
        /*!
         * @brief Protocol 1.0
         * @param id: The motor id
         * @param acceleration: Goal acceleration
         * @return If succeed to send the acceleration command or not
         */
        bool setMotorAccelerationProtocol1(uint8_t id, int32_t acceleration);
        /*!
         * @brief Protocol 2.0
         * @param id: The motor id
         * @param acceleration: Goal acceleration
         * @return If succeed to send the acceleration command or not
         */
        bool setMotorAccelerationProtocol2(uint8_t id, int32_t acceleration);
        /*!
         * @brief Protocol 1.0
         * @param id: The motor id
         * @param position: Position in ticks that return from the motor
         * @return If succeed to send the present position command or not
         */
        bool getMotorPositionProtocol1(uint8_t id, int32_t &position);
        /*!
         * @brief Protocol 2.0
         * @param id: The motor id
         * @param position: Position in ticks that return from the motor
         * @return If succeed to send the present position command or not
         */
        bool getMotorPositionProtocol2(uint8_t id, int32_t &position);
        /*!
         * @brief Protocol 1.0
         * @param id: The motor id
         * @param speed: Speed that return from the motor
         * @return If succeed to send the present speed command or not
         */
        bool getMotorSpeedProtocol1(uint8_t id, int32_t &speed);
        /*!
         * @brief Protocol 2.0
         * @param id: The motor id
         * @param speed: Speed that return from the motor
         * @return If succeed to send the present speed command or not
         */
        bool getMotorSpeedProtocol2(uint8_t id, int32_t &speed);
        /*!
         * @brief Protocol 1.0
         * @param id: The motor id
         * @param load: Load that return from the motor
         * @return If succeed to send the present load command or not
         */
        bool getMotorLoadProtocol1(uint8_t id, int16_t &load);
        /*!
         * @brief Protocol 2.0
         * @param id: The motor id
         * @param load: Load that return from the motor
         * @return If succeed to send the present load command or not
         */
        bool getMotorLoadProtocol2(uint8_t id, int16_t &load);
        /*!
         * @brief Protocol 1.0
         * @param id: The motor id
         * @param model: The motor model
         * @return If succeed to send the read model command or not
         */
        bool getMotorModelProtocol1(uint8_t id, uint16_t &model);
        /*!
         * @brief Protocol 2.0
         * @param id: The motor id
         * @param model: The motor model
         * @return If succeed to send the read model command or not
         */
        bool getMotorModelProtocol2(uint8_t id, uint16_t &model);

    public:
        DynamixelDriver(const char *port, unsigned int baudrate, DriverMode mode);

        ~DynamixelDriver();
        /*!
         * @return If the communication port is open or not.
         */
        bool isPortOpen();
        /*!
         * @brief Method that ping the motors.
         * @param motorInfo: The current motor info
         * @return If the motor return ping or not.
         */
        bool pingMotor(DxlMotorInfo_t motorInfo);
        /*!
         * @param motorInfo: The current motor info
         * @param torque: Disable/Enable motor tourqe
         * @return If succeed to enable torque or disable torque
         */
        bool setMotorTorque(DxlMotorInfo_t motorInfo, uint8_t torque);
        /*!
         * @param motorInfo: The current motor info
         * @param ticks: Position in ticks
         * @return If succeed to send the speed command or not
         */
        bool setMotorPosition(DxlMotorInfo_t motorInfo, int32_t ticks);
        /*!
         * @param motorInfo: The current motor info
         * @param speed: The motor goal speed
         * @return If succeed to send the speed command or not
         */
        bool setMotorSpeed(DxlMotorInfo_t motorInfo, int32_t speed);

        bool setMotorAcceletarion(DxlMotorInfo_t motorInfo, int32_t acceleration);
        /*!
         * @param motorInfo: The current motor info
         * @param position: Position in ticks that return from the motor
         * @return If succeed to send the present speed command or not
         */
        bool getMotorPosition(DxlMotorInfo_t motorInfo, int32_t &position);
        /*!
         * @param motorInfo: The current motor info
         * @param speed: Speed that return from the motor
         * @return If succeed to send the present speed command or not
         */
        bool getMotorSpeed(DxlMotorInfo_t motorInfo, int32_t &speed);
        /*!
         * @param motorInfo: The current motor info
         * @param load: Load that return from the motor
         * @return If succeed to send the present load command or not
         */
        bool getMotorLoad(DxlMotorInfo_t motorInfo, int16_t &load);
        /*!
         * @param motorInfo: The current motor info
         * @param model: The motor model
         * @return If succeed to send the read model command or not
         */
        bool getMotorModel(DxlMotorInfo_t motorInfo, uint16_t &model);
    };
}

#endif //ROBOTICAN_HARDWARE_INTERFACE_DYNAMIXEL_DRIVER_H
