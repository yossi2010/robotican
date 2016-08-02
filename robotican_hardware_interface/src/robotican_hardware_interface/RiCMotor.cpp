//
// Created by tom on 15/05/16.
//

#include <robotican_hardware_interface/RiCMotor.h>
#include <robotican_hardware_interface/RiCBoardPotentiometerConfig.h>
#include <robotican_hardware_interface/RiCBoardConfig.h>

namespace robotican_hardware {

    void RiCMotor::deviceAck(const DeviceAck *ack) {
        Device::deviceAck(ack);
        if(isReady()) {
            ros_utils::rosInfo("Motor is ready");
        }
        else {
            ros_utils::rosError("RiCBoard can't build motor object for some reason, this program will shut down now");
            ros::shutdown();
        }
    }

    RiCMotor::RiCMotor(byte id, TransportLayer *transportLayer, byte motorAddress, byte eSwitchPin, byte eSwitchType)
            : Device(id, transportLayer) {
        _motorAddress = motorAddress;
        _eSwitchPin = eSwitchPin;
        _eSwitchType = eSwitchType;
    }



    CloseLoopMotor::CloseLoopMotor(byte id, TransportLayer *transportLayer, byte motorAddress, byte eSwitchPin, byte eSwitchType,
                                   CloseMotorType::CloseMotorType motorType, CloseMotorMode::CloseMotorMode mode)
            : RiCMotor(id, transportLayer, motorAddress, eSwitchPin, eSwitchType) {
        _motorType = motorType;
        _mode = mode;
        _lastCmd = 0.0;

    }

    JointInfo_t *CloseLoopMotor::getJointInfo() {
        return &_jointInfo;
    }

    void CloseLoopMotor::update(const DeviceMessage *deviceMessage) {
        if(isReady()) {

            MotorFeedback *feedback = (MotorFeedback *) deviceMessage;
            _jointInfo.position = feedback->rad;
            _jointInfo.velocity = feedback->rad_s;
        }
    }

    void CloseLoopMotor::write() {
        if(isReady()) {
            if(checkIfLastCmdChange()) {
                MotorSetPoint point;
                point.length = sizeof(point);
                point.checkSum = 0;
                point.id = getId();
                point.point = (float) _jointInfo.cmd;

                uint8_t *rawData = (uint8_t *) &point;
                point.checkSum = _transportLayer->calcChecksum(rawData, point.length);
                _transportLayer->write(rawData, point.length);
                _lastCmd = (float) _jointInfo.cmd;
            }
        }
    }

    bool CloseLoopMotor::checkIfLastCmdChange() {
        float delta = fabsf((float) (_jointInfo.cmd - _lastCmd));
        return delta >= MOTOR_EPSILON;
    }

    CloseMotorType::CloseMotorType CloseLoopMotor::getCloseMotorType() {
        return _motorType;
    }

    CloseMotorMode::CloseMotorMode CloseLoopMotor::getMode() {
        return _mode;
    }

    byte RiCMotor::getESwitchPin() {
        return _eSwitchPin;
    }

    byte RiCMotor::getESwitchType() {
        return _eSwitchType;
    }

    byte RiCMotor::getAddress() {
        return _motorAddress;
    }

    OpenLoopMotor::OpenLoopMotor(byte id, TransportLayer *transportLayer, byte motorAddress, byte eSwitchPin, byte eSwitchType,
                                     float maxSetPointSpeed, float minSetPointSpeed, byte encoderA, byte encoderB, int8_t motorDirection,
                                     int8_t encoderDirection, uint16_t LPFHzSpeed, uint16_t LPFHzInput, float LPFAlphaInput,
                                     float LPFAlphaSpeed, uint16_t PPR)
            : RiCMotor(id, transportLayer, motorAddress, eSwitchPin, eSwitchType) {

        _maxSetPointSpeed = maxSetPointSpeed;
        _minSetPointSpeed = minSetPointSpeed;
        _encoderPinA = encoderA;
        _encoderPinB = encoderB;
        _motorDirection = motorDirection;
        _encoderDirection = encoderDirection;
        _LPFHzSpeed = LPFHzSpeed;
        _LPFHzInput = LPFHzInput;
        _LPFAlphaInput = LPFAlphaInput;
        _PPR = PPR;
        _LPFAlphaSpeed = LPFAlphaSpeed;
    }

    void OpenLoopMotor::update(const DeviceMessage *deviceMessage) {

        MotorFeedback *feedback = (MotorFeedback *) deviceMessage;
        _jointInfo.position = feedback->rad;
        _jointInfo.velocity = feedback->rad_s;

    }

    void OpenLoopMotor::write() {
        if(isReady()) {
            MotorSetPoint motorSetPoint;
            motorSetPoint.length = sizeof(motorSetPoint);
            motorSetPoint.checkSum = 0;
            motorSetPoint.id = getId();
            motorSetPoint.point = (float) _jointInfo.cmd;


            uint8_t *rawData = (uint8_t *) &motorSetPoint;
            motorSetPoint.checkSum = _transportLayer->calcChecksum(rawData, motorSetPoint.length);
            _transportLayer->write(rawData, motorSetPoint.length);
        }
    }


    void OpenLoopMotor::buildDevice() {
        BuildMotorOpenLoop buildMotorOpenLoop;
        buildMotorOpenLoop.length = sizeof(buildMotorOpenLoop);
        buildMotorOpenLoop.checkSum = 0;
        buildMotorOpenLoop.id = getId();
        buildMotorOpenLoop.motorAddress = getAddress();
        buildMotorOpenLoop.motorDirection = _motorDirection;
        buildMotorOpenLoop.eSwitchPin = getESwitchPin();
        buildMotorOpenLoop.eSwitchType = getESwitchType();
        buildMotorOpenLoop.maxSetPointSpeed = _maxSetPointSpeed;
        buildMotorOpenLoop.minSetPointSpeed = _minSetPointSpeed;
        buildMotorOpenLoop.encoderPinA = _encoderPinA;
        buildMotorOpenLoop.encoderPinB = _encoderPinB;
        buildMotorOpenLoop.encoderDirection = _encoderDirection;
        buildMotorOpenLoop.PPR = _PPR;
        buildMotorOpenLoop.LPFAlphaSpeed = _LPFAlphaSpeed;
        buildMotorOpenLoop.LPFHzSpeed = _LPFHzSpeed;
        buildMotorOpenLoop.LPFHzInput = _LPFHzInput;
        buildMotorOpenLoop.LPFAlphaInput = _LPFAlphaInput;


        uint8_t* rawData = (uint8_t*) &buildMotorOpenLoop;
        buildMotorOpenLoop.checkSum = _transportLayer->calcChecksum(rawData, buildMotorOpenLoop.length);
        _transportLayer->write(rawData, buildMotorOpenLoop.length);
    }

    JointInfo_t *OpenLoopMotor::getJointInfo() {
        return &_jointInfo;
    }

    CloseLoopMotorWithEncoder::CloseLoopMotorWithEncoder(byte id, TransportLayer *transportLayer, byte motorAddress, byte eSwitchPin,
                                                         byte eSwitchType, CloseMotorType::CloseMotorType motoryType,
                                                         CloseMotorMode::CloseMotorMode mode, CloseMotorWithEncoderParam param,
                                                         std::string jointName)
            : CloseLoopMotor(id, transportLayer, motorAddress, eSwitchPin, eSwitchType, motoryType, mode),
              _mutex() ,_nodeHandle((std::string("~/") + jointName)), _server(_mutex , _nodeHandle) , _spinner(1){
        _callbackType = boost::bind(&CloseLoopMotorWithEncoder::dynamicCallback, this, _1, _2);

        _server.setCallback(_callbackType);
        _spinner.start();
        _timer = _nodeHandle.createTimer(ros::Duration(0.02), &CloseLoopMotorWithEncoder::timerCallback, this);
        _timer.start();
        _statusPub = _nodeHandle.advertise<std_msgs::Float64>("motor_current_set_point", 10);
        robotican_hardware_interface::RiCBoardConfig config;
        config.motor_speed_lpf_hz = param.LPFHzSpeed;
        config.motor_speed_lpf_alpha = param.LPFAlphaSpeed;
        config.motor_input_lpf_hz = param.LPFHzInput;
        config.motor_input_lpf_alpha = param.LPFAlphaInput;
        config.motor_pid_hz = param.PIDHz;
        config.motor_kp = param.KP;
        config.motor_ki = param.KI;
        config.motor_kd = param.KD;

        _server.updateConfig(config);

        _params = param;
        _isSetParam = false;


    }


    void CloseLoopMotorWithEncoder::buildDevice() {
        BuildMotorCloseLoopWithEncoder buildMotorCloseLoop;
        buildMotorCloseLoop.length = sizeof(buildMotorCloseLoop);
        buildMotorCloseLoop.checkSum = 0;
        buildMotorCloseLoop.id = getId();
        buildMotorCloseLoop.motorAddress = getAddress();
        buildMotorCloseLoop.eSwitchPin = getESwitchPin();
        buildMotorCloseLoop.eSwitchType = getESwitchType();
        buildMotorCloseLoop.encoderPinA = _params.encoderPinA;
        buildMotorCloseLoop.encoderPinB = _params.encoderPinB;
        buildMotorCloseLoop.LPFHzSpeed = _params.LPFHzSpeed;
        buildMotorCloseLoop.LPFHzInput = _params.LPFHzInput;
        buildMotorCloseLoop.PIDHz = _params.PIDHz;
        buildMotorCloseLoop.PPR = _params.PPR;
        buildMotorCloseLoop.timeout = _params.timeout;
        buildMotorCloseLoop.motorDirection = _params.motorDirection;
        buildMotorCloseLoop.encoderDirection = _params.encoderDirection;
        buildMotorCloseLoop.stopLimitMax = _params.stopLimitMax;
        buildMotorCloseLoop.stopLimitMin = _params.stopLimitMin;

        buildMotorCloseLoop.LPFAlphaSpeed = _params.LPFAlphaSpeed;
        buildMotorCloseLoop.LPFAlphaInput = _params.LPFAlphaInput;
        buildMotorCloseLoop.KP = _params.KP;
        buildMotorCloseLoop.KI = _params.KI;
        buildMotorCloseLoop.KD = _params.KD;
        buildMotorCloseLoop.maxSetPointSpeed = _params.maxSetPointSpeed;
        buildMotorCloseLoop.minSetPointSpeed = _params.minSetPointSpeed;
        buildMotorCloseLoop.maxSetPointPos = _params.maxSetPointPos;
        buildMotorCloseLoop.minSetPointPos = _params.minSetPointPos;
        buildMotorCloseLoop.limit = _params.limit;
        buildMotorCloseLoop.motorType = getCloseMotorType();
        buildMotorCloseLoop.motorMode = getMode();
        buildMotorCloseLoop.baisMin = _params.baisMin;
        buildMotorCloseLoop.baisMax = _params.baisMax;

        uint8_t* rawData = (uint8_t*) &buildMotorCloseLoop;
        buildMotorCloseLoop.checkSum = _transportLayer->calcChecksum(rawData, buildMotorCloseLoop.length);
        _transportLayer->write(rawData, buildMotorCloseLoop.length);
    }

    void CloseLoopMotorWithEncoder::setParams(uint16_t speedLpfHz, uint16_t inputLpfHz, uint16_t pidHz, float speedLpfAlpha, float inputLpfAlpha, float KP,
                                                  float KI, float KD) {
        _isSetParam = true;
        _params.LPFHzSpeed = speedLpfHz;
        _params.PIDHz = pidHz;
        _params.LPFAlphaSpeed = speedLpfAlpha;
        _params.LPFHzInput = inputLpfHz;
        _params.LPFAlphaInput = inputLpfAlpha;
        _params.KP = KP;
        _params.KI = KI;
        _params.KD = KD;

    }

    void CloseLoopMotorWithEncoder::write() {
        CloseLoopMotor::write();
        if(isReady()) {
            if (_isSetParam) {
                _isSetParam = false;
                SetMotorParam param;
                param.length = sizeof(param);
                param.checkSum = 0;
                param.id = getId();
                param.speedLpfHz = _params.LPFHzSpeed;
                param.pidHz = _params.PIDHz;
                param.speedLfpAlpha = _params.LPFAlphaSpeed;
                param.KP = _params.KP;
                param.KI = _params.KI;
                param.KD = _params.KD;
                param.inputLfpAlpha = 0;
                param.inputLpfHz = _params.LPFHzInput;
                param.inputLfpAlpha = _params.LPFAlphaInput;

                uint8_t *rawData = (uint8_t *) &param;

                param.checkSum = _transportLayer->calcChecksum(rawData, param.length);
                _transportLayer->write(rawData, param.length);
            }
        }
    }

    void CloseLoopMotorWithEncoder::dynamicCallback(robotican_hardware_interface::RiCBoardConfig &config, uint32_t level) {
        setParams((uint16_t) config.motor_speed_lpf_hz, config.motor_input_lpf_hz, (uint16_t) config.motor_pid_hz,
                  (float) config.motor_speed_lpf_alpha, config.motor_input_lpf_alpha, (float) config.motor_kp, (float) config.motor_ki,
                  (float) config.motor_kd);
    }

    void CloseLoopMotorWithEncoder::timerCallback(const ros::TimerEvent &e) {
        std_msgs::Float64 msg;
        msg.data = _jointInfo.cmd;

        _statusPub.publish(msg);
    }

    void CloseLoopMotorWithPotentiometer::setParams(uint16_t speedLpfHz, uint16_t inputLpfHz, uint16_t pidHz, float speedLpfAlpha, float inputLpfAlpha, float KP,
                                                    float KI, float KD) {

    }

    void CloseLoopMotorWithPotentiometer::buildDevice() {
        BuildCloseLoopWithPotentiometer buildCloseLoopWithPotentiometer;
        buildCloseLoopWithPotentiometer.length = sizeof(buildCloseLoopWithPotentiometer);
        buildCloseLoopWithPotentiometer.checkSum = 0;
        buildCloseLoopWithPotentiometer.id = getId();

        buildCloseLoopWithPotentiometer.motorAddress = getAddress();
        buildCloseLoopWithPotentiometer.motorMode = getMode();
        buildCloseLoopWithPotentiometer.eSwitchPin = getESwitchPin();
        buildCloseLoopWithPotentiometer.eSwitchType = getESwitchType();
        buildCloseLoopWithPotentiometer.motorType = getCloseMotorType();
        buildCloseLoopWithPotentiometer.LPFHzSpeed = _param.LPFHzSpeed;
        buildCloseLoopWithPotentiometer.PIDHz = _param.PIDHz;
        buildCloseLoopWithPotentiometer.PPR = _param.PPR;
        buildCloseLoopWithPotentiometer.timeout = _param.timeout;
        buildCloseLoopWithPotentiometer.motorDirection = _param.motorDirection;
        buildCloseLoopWithPotentiometer.encoderDirection = _param.encoderDirection;
        buildCloseLoopWithPotentiometer.LPFAlphaSpeed = _param.LPFAlphaSpeed;
        buildCloseLoopWithPotentiometer.KP = _param.KP;
        buildCloseLoopWithPotentiometer.KI = _param.KI;
        buildCloseLoopWithPotentiometer.KD = _param.KD;
        buildCloseLoopWithPotentiometer.maxSetPointSpeed = _param.maxSetPointSpeed;
        buildCloseLoopWithPotentiometer.minSetPointSpeed = _param.minSetPointSpeed;
        buildCloseLoopWithPotentiometer.maxSetPointPos = _param.maxSetPointPos;
        buildCloseLoopWithPotentiometer.minSetPointPos = _param.minSetPointPos;
        buildCloseLoopWithPotentiometer.limit = _param.limit;
        buildCloseLoopWithPotentiometer.a = _param.a;
        buildCloseLoopWithPotentiometer.b = _param.b;
        buildCloseLoopWithPotentiometer.pin = _param.pin;
        buildCloseLoopWithPotentiometer.tolerance = _param.tolerance;
        buildCloseLoopWithPotentiometer.stopLimitMax = _param.stopLimitMax;
        buildCloseLoopWithPotentiometer.stopLimitMin = _param.stopLimitMin;
        buildCloseLoopWithPotentiometer.LPFHzInput = _param.LPFHzInput;
        buildCloseLoopWithPotentiometer.LPFAlphaInput = _param.LPFAlphaInput;

        uint8_t* rawData = (uint8_t*)&buildCloseLoopWithPotentiometer;
        buildCloseLoopWithPotentiometer.checkSum = _transportLayer->calcChecksum(rawData, buildCloseLoopWithPotentiometer.length);
        _transportLayer->write(rawData, buildCloseLoopWithPotentiometer.length);

    }

    CloseLoopMotorWithPotentiometer::CloseLoopMotorWithPotentiometer(byte id, TransportLayer *transportLayer,
                                                                         byte motorAddress, byte eSwitchPin,
                                                                         byte eSwitchType,
                                                                         CloseMotorType::CloseMotorType motorType,
                                                                         CloseMotorMode::CloseMotorMode mode,
                                                                         CloseMotorWithPotentiometerParam motorParam,
                                                                         std::string jointName)
            : CloseLoopMotor(id, transportLayer, motorAddress, eSwitchPin, eSwitchType, motorType, mode)
            , _mutex() ,_nodeHandle((std::string("~/") + jointName)), _server(_mutex , _nodeHandle) {
        _callbackType = boost::bind(&CloseLoopMotorWithPotentiometer::dynamicCallback, this, _1, _2);

        _server.setCallback(_callbackType);
        robotican_hardware_interface::RiCBoardPotentiometerConfig config;
        config.motor_speed_lpf_hz = motorParam.LPFHzSpeed;
        config.motor_input_lpf_hz = motorParam.LPFHzInput;
        config.motor_pid_hz = motorParam.PIDHz;
        config.motor_speed_lpf_alpha = motorParam.LPFAlphaSpeed;
        config.motor_input_lpf_alpha = motorParam.LPFAlphaInput;
        config.motor_kp = motorParam.KP;
        config.motor_ki = motorParam.KI;
        config.motor_kd = motorParam.KD;
        config.motor_a = motorParam.a;
        config.motor_b = motorParam.b;
        config.motor_tolerance = motorParam.tolerance;

        _server.updateConfig(config);


        _param = motorParam;
        _isParamChange = false;
        _firstTime = true;
    }

    void CloseLoopMotorWithPotentiometer::setParams(uint16_t speedLpfHz, uint16_t inputLpfHz, uint16_t pidHz, float speedLpfAlpha, float inputLpfAlpha, float KP,
                                                        float KI, float KD, float a, float b, float tolerance) {
        _param.LPFHzSpeed = speedLpfHz;
        _param.LPFHzInput = inputLpfHz;
        _param.PIDHz = pidHz;
        _param.LPFAlphaSpeed = speedLpfAlpha;
        _param.LPFAlphaInput = inputLpfAlpha;
        _param.KP = KP;
        _param.KI = KI;
        _param.KD = KD;
        _param.a = a;
        _param.b = b;
        _param.tolerance = tolerance;
        _isParamChange = true;

    }

    void CloseLoopMotorWithPotentiometer::dynamicCallback(robotican_hardware_interface::RiCBoardPotentiometerConfig &config, uint32_t level) {
        setParams((uint16_t) config.motor_speed_lpf_hz, (uint16_t) config.motor_input_lpf_hz, (uint16_t) config.motor_pid_hz,
                  (float) config.motor_speed_lpf_alpha, (float) config.motor_input_lpf_alpha, (float) config.motor_kp, (float) config.motor_ki,
                  (float) config.motor_kd, (float) config.motor_a, (float) config.motor_b,
                  (float) config.motor_tolerance);
    }


    void CloseLoopMotorWithPotentiometer::write() {
        if(!_firstTime) {
            CloseLoopMotor::write();
        }
        if(_isParamChange) {
            _isParamChange = false;
            SetCloseMotorWithPotentiometer setCloseMotorWithPotentiometer;
            setCloseMotorWithPotentiometer.length = sizeof(setCloseMotorWithPotentiometer);
            setCloseMotorWithPotentiometer.checkSum = 0;
            setCloseMotorWithPotentiometer.id = getId();

            setCloseMotorWithPotentiometer.speedLpfHz = _param.LPFHzSpeed;
            setCloseMotorWithPotentiometer.inputLpfHz = _param.LPFHzInput;
            setCloseMotorWithPotentiometer.pidHz = _param.PIDHz;
            setCloseMotorWithPotentiometer.speedLfpAlpha = _param.LPFAlphaSpeed;
            setCloseMotorWithPotentiometer.inputLfpAlpha = _param.LPFAlphaInput;
            setCloseMotorWithPotentiometer.KP = _param.KP;
            setCloseMotorWithPotentiometer.KI = _param.KI;
            setCloseMotorWithPotentiometer.KD = _param.KD;
            setCloseMotorWithPotentiometer.a = _param.a;
            setCloseMotorWithPotentiometer.b = _param.b;
            setCloseMotorWithPotentiometer.tolerance = _param.tolerance;

            uint8_t *rawData = (uint8_t*)&setCloseMotorWithPotentiometer;

            setCloseMotorWithPotentiometer.checkSum = _transportLayer->calcChecksum(rawData, setCloseMotorWithPotentiometer.length);
            _transportLayer->write(rawData, setCloseMotorWithPotentiometer.length);

        }

    }


    void CloseLoopMotorWithPotentiometer::update(const DeviceMessage *deviceMessage) {
        CloseLoopMotor::update(deviceMessage);
        if(isReady() && _firstTime) {
            _jointInfo.cmd = _jointInfo.position;
            _firstTime = false;
        }
    }
}
