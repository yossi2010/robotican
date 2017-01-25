//
// Created by tom on 15/05/16.
//

#ifndef ROBOTICAN_HARDWARE_INTERFACE_MOTOR_H
#define ROBOTICAN_HARDWARE_INTERFACE_MOTOR_H


#include <dynamic_reconfigure/server.h>
#include <std_msgs/Float64.h>
#include <robotican_hardware_interface/Device.h>
#include <robotican_hardware_interface/ros_utils.h>
#include <robotican_hardware_interface/RiCBoardManager.h>
#include <robotican_hardware_interface/RiCBoardConfig.h>
#include <robotican_hardware_interface/RiCBoardPotentiometerConfig.h>

#define MOTOR_EPSILON 0.001

namespace robotican_hardware {


    class RiCMotor : public Device {
    private:
        byte _motorAddress;
        byte _eSwitchPin;
        byte _eSwitchType;
    public:
        RiCMotor(byte id, TransportLayer *transportLayer, byte motorAddress, byte eSwitchPin, byte eSwitchType);

        virtual void deviceAck(const DeviceAck *ack);

        virtual void update(const DeviceMessage *deviceMessage)=0;

        virtual void write()=0;

        virtual void buildDevice()=0;

    protected:
        byte getESwitchPin();
        byte getESwitchType();
        byte getAddress();

    };

    class OpenLoopMotor : public RiCMotor {
    private:
        JointInfo_t _jointInfo;
        byte _encoderPinA;
        byte _encoderPinB;
        int8_t _motorDirection;
        int8_t _encoderDirection;
        uint16_t _LPFHzSpeed;
        uint16_t _LPFHzInput;
        uint16_t _PPR;
        float _maxSetPointSpeed;
        float _minSetPointSpeed;
        float _LPFAlphaSpeed;
        float _LPFAlphaInput;

    public:

        OpenLoopMotor(byte id, TransportLayer *transportLayer, byte motorAddress, byte eSwitchPin, byte eSwitchType,
                              float maxSetPointSpeed, float minSetPointSpeed, byte encoderA, byte encoderB, int8_t motorDirection,
                              int8_t encoderDirection, uint16_t LPFHzSpeed, uint16_t LPFHzInput, float LPFAlphaInput,
                              float LPFAlphaSpeed, uint16_t PPR);

        virtual void update(const DeviceMessage *deviceMessage);

        virtual void write();

        JointInfo_t* getJointInfo();


        virtual void buildDevice();
    };

    struct CloseMotorParams {
        uint16_t LPFHzSpeed;
        uint16_t PIDHz;
        uint16_t PPR;
        uint16_t timeout;
        int8_t motorDirection;
        int8_t encoderDirection;
        float LPFAlphaSpeed;
        float KP;
        float KI;
        float KD;
        float KFF;
        float maxSetPointSpeed;
        float limit;
        float stopLimitMax;
        float stopLimitMin;
    };

    struct  CloseMotorWithEncoderParam : CloseMotorParams {
        byte encoderPinA;
        byte encoderPinB;
        int16_t baisMin;
        int16_t baisMax;
        float LPFAlphaInput;
        uint16_t LPFHzInput;
        float maxSetPointPos;
        float minSetPointSpeed;
        float minSetPointPos;
    };

    struct CloseMotorWithPotentiometerParam : CloseMotorParams {
        byte pin;
        float a;
        float b;
        float tolerance;
        uint16_t LPFHzInput;
        float LPFAlphaInput;
        float maxSetPointPos;
        float minSetPointSpeed;
        float minSetPointPos;
        float K;
        uint16_t toleranceTime;
    };

    class CloseLoopMotor : public RiCMotor {
    private:
        float _lastCmd;
        CloseMotorType::CloseMotorType _motorType;
        CloseMotorMode::CloseMotorMode _mode;
        bool checkIfLastCmdChange();
    public:
        CloseLoopMotor(byte id, TransportLayer *transportLayer, byte motorAddress, byte eSwitchPin, byte eSwitchType,
                               CloseMotorType::CloseMotorType motorType, CloseMotorMode::CloseMotorMode mode);
        JointInfo_t* getJointInfo();

        virtual void update(const DeviceMessage *deviceMessage);

        virtual void write();

        virtual void setParams(uint16_t speedLpfHz, uint16_t inputLpfHz, uint16_t pidHz, float speedLpfAlpha,
                                       float inputLpfAlpha, float KP, float KI, float KD, float KFF, float limit)=0;

        virtual void buildDevice() = 0;

    protected:
        JointInfo_t _jointInfo;
        CloseMotorType::CloseMotorType getCloseMotorType();
        CloseMotorMode::CloseMotorMode getMode();

    };

    class CloseLoopMotorWithEncoder : public CloseLoopMotor {
    private:
        CloseMotorWithEncoderParam _params;
        ros::NodeHandle _nodeHandle;
        boost::recursive_mutex _mutex;
        ros::AsyncSpinner _spinner;
        ros::Publisher _statusPub;
        ros::Timer _timer;
        bool _isSetParam;
        //Dynamic param setting
        dynamic_reconfigure::Server <robotican_hardware_interface::RiCBoardConfig> _server;
        dynamic_reconfigure::Server<robotican_hardware_interface::RiCBoardConfig>::CallbackType _callbackType;
        void dynamicCallback(robotican_hardware_interface::RiCBoardConfig &config, uint32_t level);
        void timerCallback(const ros::TimerEvent& e);
        virtual void setParams(uint16_t speedLpfHz, uint16_t inputLpfHz, uint16_t pidHz, float speedLpfAlpha,
                                       float inputLpfAlpha, float KP, float KI, float KD, float KFF, float limit);

    public:
        CloseLoopMotorWithEncoder(byte id, TransportLayer *transportLayer, byte motorAddress, byte eSwitchPin,
                                          byte eSwitchType, CloseMotorType::CloseMotorType motoryType,
                                          CloseMotorMode::CloseMotorMode mode, CloseMotorWithEncoderParam param,
                                          std::string jointName);
        virtual void buildDevice();
        virtual void write();

    };

    class CloseLoopMotorWithPotentiometer : public CloseLoopMotor {
    private:
        bool _isParamChange;
        CloseMotorWithPotentiometerParam _param;
        ros::NodeHandle _nodeHandle;
        bool _firstTime;
        boost::recursive_mutex _mutex;
        //Dynamic param setting
        dynamic_reconfigure::Server <robotican_hardware_interface::RiCBoardPotentiometerConfig> _server;
        dynamic_reconfigure::Server<robotican_hardware_interface::RiCBoardPotentiometerConfig>::CallbackType _callbackType;
        void dynamicCallback(robotican_hardware_interface::RiCBoardPotentiometerConfig &config, uint32_t level);

    public:
        CloseLoopMotorWithPotentiometer(byte id, TransportLayer *transportLayer,
                                                byte motorAddress, byte eSwitchPin,
                                                byte eSwitchType,
                                                CloseMotorType::CloseMotorType motorType,
                                                CloseMotorMode::CloseMotorMode mode,
                                                CloseMotorWithPotentiometerParam motorParam,
                                                std::string jointName);

        virtual void setParams(uint16_t speedLpfHz, uint16_t inputLpfHz, uint16_t pidHz, float speedLpfAlpha,
                               float inputLpfAlpha, float KP, float KI, float KD, float KFF, float limit);

        virtual void setParams(uint16_t speedLpfHz, uint16_t inputLpfHz, uint16_t pidHz, float speedLpfAlpha, float inputLpfAlpha, float KP,
                                       float KI, float KD, float KFF, float K, float a, float b, float tolerance, uint16_t toleranceTime,
                                       float limit);

        virtual void buildDevice();

        virtual void update(const DeviceMessage *deviceMessage);

        virtual void write();
    };

}

#endif //ROBOTICAN_HARDWARE_INTERFACE_MOTOR_H
