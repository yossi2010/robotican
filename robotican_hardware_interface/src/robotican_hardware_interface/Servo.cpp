//
// Created by tom on 10/05/16.
//

#include <robotican_hardware_interface/Servo.h>

namespace robotican_hardware {

    Servo::Servo(byte id, TransportLayer *transportLayer, byte pin, float a, float b, float max, float min,
                     float initPos, std::string jointName) : Device(id , transportLayer),
                                                             _mutex(), _nodeHandle((std::string("~/") + jointName)), _server(_mutex, _nodeHandle) {
        _callbackType = boost::bind(&Servo::dynamicCallback, this, _1, _2);
        _server.setCallback(_callbackType);
        _pin = pin;
        _a = a;
        _b = b;
        _max = max;
        _min = min;
        _lastCmd = 0.0;
        _isChangeParam = false;


        robotican_hardware_interface::RiCBoardServoConfig config;
        config.A = _a;
        config.B = _b;
        config.max = _max;
        config.min = _min;

        _server.updateConfig(config);


    }




    void Servo::update(const DeviceMessage *deviceMessage) {
        if(isReady()) {
            ServoFeedback *feedback = (ServoFeedback *) deviceMessage;
            _jointInfo.position = feedback->pos;
        }
    }

    void Servo::write() {
        if(isReady()) {
            if(checkIfLastCmdChange()) {
                ServoSetPoint point;
                point.length = sizeof(point);
                point.checkSum = 0;
                point.id = getId();
                point.pos = (float) _jointInfo.cmd;

                uint8_t *rawData = (uint8_t *) &point;
                point.checkSum = _transportLayer->calcChecksum(rawData, point.length);
                _transportLayer->write(rawData, point.length);
                _lastCmd = (float) _jointInfo.cmd;
            }
            if(_isChangeParam) {
                _isChangeParam = false;
                SetServoParam param;
                param.length = sizeof(param);
                param.checkSum = 0;
                param.id = getId();
                param.a = _a;
                param.b = _b;
                param.max = _max;
                param.min = _min;

                uint8_t *rawData = (uint8_t*) &param;
                param.checkSum = _transportLayer->calcChecksum(rawData, param.length);
                _transportLayer->write(rawData, param.length);
            }
        }
    }

    JointInfo_t* Servo::getJointInfo() {
        return &_jointInfo;
    }

    void Servo::buildDevice() {
        BuildServo buildServo;
        buildServo.length = sizeof(buildServo);
        buildServo.checkSum = 0;
        buildServo.id = getId();
        buildServo.pin = _pin;
        buildServo.a = _a;
        buildServo.b = _b;
        buildServo.max = _max;
        buildServo.min = _min;

        uint8_t* rawData = (uint8_t*)&buildServo;
        buildServo.checkSum = _transportLayer->calcChecksum(rawData, buildServo.length);
        _transportLayer->write(rawData, buildServo.length);
    }


    void Servo::deviceAck(const DeviceAck *ack) {
        Device::deviceAck(ack);
        if(isReady()) {
            ros_utils::rosInfo("Servo is ready");
        }
        else {
            ros_utils::rosError("Can't build Servo for some reason");
            ros::shutdown();
        }
    }

    bool Servo::checkIfLastCmdChange() {
        float delta = fabsf((float) (_jointInfo.cmd - _lastCmd));
        return delta >= SERVO_EPSILON;
    }

    void Servo::setParam(float a, float b, float max, float min) {
        _isChangeParam = true;
        _a = a;
        _b = b;
        _max = max;
        _min = min;

    }

    void Servo::dynamicCallback(robotican_hardware_interface::RiCBoardServoConfig &config, uint32_t level) {
        setParam((float) config.A, (float) config.B, (float) config.max, (float) config.min);

    }
}