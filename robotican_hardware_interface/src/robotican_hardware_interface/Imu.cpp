//
// Created by tom on 15/05/16.
//

#include <robotican_hardware_interface/Imu.h>

namespace robotican_hardware {

    void Imu::update(const DeviceMessage *deviceMessage) {
        if(isReady()) {
            if(deviceMessage->deviceMessageType == DeviceMessageType::ImuFeedback) {
                ImuFeedback *feedback = (ImuFeedback *) deviceMessage;

                sensor_msgs::Imu imuMsg;
                imuMsg.header.frame_id = _frameId;
                imuMsg.header.stamp = ros::Time::now();
                imuMsg.orientation.x = feedback->orientationX;
                imuMsg.orientation.y = feedback->orientationY;
                imuMsg.orientation.z = feedback->orientationZ;
                imuMsg.orientation.w = feedback->orientationW;
                imuMsg.linear_acceleration.x = (feedback->accelerationX * _imuLinearAccFix[0][0]
                                                +  feedback->accelerationY * _imuLinearAccFix[0][1]
                                                + feedback->accelerationZ * _imuLinearAccFix[0][2]) * G2ms;
                imuMsg.linear_acceleration.y = (feedback->accelerationX * _imuLinearAccFix[1][0]
                                               +  feedback->accelerationY * _imuLinearAccFix[1][1]
                                               + feedback->accelerationZ * _imuLinearAccFix[1][2]) * G2ms;
                imuMsg.linear_acceleration.z = (feedback->accelerationX * _imuLinearAccFix[2][0]
                                                +  feedback->accelerationY * _imuLinearAccFix[2][1]
                                                + feedback->accelerationZ * _imuLinearAccFix[2][2]) * G2ms;
                imuMsg.angular_velocity.x = (feedback->velocityX * _imuAngularVelocityFix[0][0]
                                             + feedback->velocityY * _imuAngularVelocityFix[0][1]
                                             + feedback->velocityZ * _imuAngularVelocityFix[0][2]) * DEGs2RADs;
                imuMsg.angular_velocity.y = (feedback->velocityX * _imuAngularVelocityFix[1][0]
                                             + feedback->velocityY * _imuAngularVelocityFix[1][1]
                                             + feedback->velocityZ * _imuAngularVelocityFix[1][2]) * DEGs2RADs;
                imuMsg.angular_velocity.z = (feedback->velocityX * _imuAngularVelocityFix[2][0]
                                             + feedback->velocityY * _imuAngularVelocityFix[2][1]
                                             + feedback->velocityZ * _imuAngularVelocityFix[2][2]) * DEGs2RADs;

                sensor_msgs::MagneticField magneticField;
                magneticField.header.frame_id = _frameId;
                magneticField.header.stamp = ros::Time::now();
                magneticField.magnetic_field.x = (feedback->magnetometerX * _imuMagnetometerFix[0][0]
                                                  + feedback->magnetometerY * _imuMagnetometerFix[0][1]
                                                  + feedback->magnetometerZ * _imuMagnetometerFix[0][2]) * MILLI_GAUSS_2_TESLAS;
                magneticField.magnetic_field.y = (feedback->magnetometerX * _imuMagnetometerFix[1][0]
                                                  + feedback->magnetometerY * _imuMagnetometerFix[1][1]
                                                  + feedback->magnetometerZ * _imuMagnetometerFix[1][2]) * MILLI_GAUSS_2_TESLAS;
                magneticField.magnetic_field.z = (feedback->magnetometerX * _imuMagnetometerFix[2][0]
                                                  + feedback->magnetometerY * _imuMagnetometerFix[2][1]
                                                  + feedback->magnetometerZ * _imuMagnetometerFix[2][2]) * MILLI_GAUSS_2_TESLAS;

                tf::Transform transform;
                tf::Quaternion quaternion;
                tf::quaternionMsgToTF(imuMsg.orientation, quaternion);
                transform.setRotation(quaternion);

                double roll, pitch, yaw;
                double newRoll, newPitch, newYaw;
                tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);


                newRoll = roll * _imuRotationFix[0][0] + pitch * _imuRotationFix[0][1] + yaw * _imuRotationFix[0][2];
                newRoll += _imuRotationOffset[0];

                newPitch = roll * _imuRotationFix[1][0] + pitch * _imuRotationFix[1][1] + yaw * _imuRotationFix[1][2];
                newPitch += _imuRotationOffset[1];

                newYaw = roll * _imuRotationFix[2][0] + pitch * _imuRotationFix[2][1] + yaw * _imuRotationFix[2][2];
                newYaw += _imuRotationOffset[2];

                quaternion.setRPY(newRoll, newPitch, newYaw);
                ROS_INFO("[%s]: roll: %.2f , pitch: %.2f, yaw: %.2f", ros::this_node::getName().c_str(), newRoll * 180 / M_PI, newPitch * 180 / M_PI, newYaw * 180 / M_PI);

                tf::quaternionTFToMsg(quaternion, imuMsg.orientation);

                _imuAMQ.publish(imuMsg);
                _imuM.publish(magneticField);
                _broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "imu_link"));

            }
            else if(deviceMessage->deviceMessageType == DeviceMessageType::ImuClibFeedback) {
                if(!_isStopClib) {
                    ImuClibFeedback *clibFeedback = (ImuClibFeedback *) deviceMessage;
                    robotican_hardware_interface::imuClib clibMsg;
                    clibMsg.max.header.frame_id = "base_link";
                    clibMsg.min.header.frame_id = "base_link";
                    clibMsg.max.header.stamp = ros::Time::now();
                    clibMsg.min.header.stamp = ros::Time::now();

                    clibMsg.max.vector.x = clibFeedback->max[0];
                    clibMsg.max.vector.y = clibFeedback->max[1];
                    clibMsg.max.vector.z = clibFeedback->max[2];

                    clibMsg.min.vector.x = clibFeedback->min[0];
                    clibMsg.min.vector.y = clibFeedback->min[1];
                    clibMsg.min.vector.z = clibFeedback->min[2];

                    _clibPub.publish(clibMsg);
                }

            }
        }

    }

    void Imu::write() {
        if(isReady()) {
            if(_isStateChange) {
                _isStateChange = false;
                ImuSetClibState state;
                state.length = sizeof(state);
                state.checkSum = 0;
                state.id = getId();

                state.state = _imuState;
                uint8_t *rawData = (uint8_t*) &state;
                state.checkSum = _transportLayer->calcChecksum(rawData, state.length);
                _transportLayer->write(rawData, state.length);
            }
        }

    }

    void Imu::buildDevice() {
        BuildImu buildImu;
        buildImu.length = sizeof(buildImu);
        buildImu.checkSum = 0;
        buildImu.id = getId();
        buildImu.fusionHz = _fusionHz;
        buildImu.enableGyro =_enableGyro;
        buildImu.fuseCompass =_fuseCompass;

        uint8_t  *rawData = (uint8_t*) &buildImu;

        buildImu.checkSum = _transportLayer->calcChecksum(rawData, buildImu.length);
        _transportLayer->write(rawData, buildImu.length);
        ros::Duration(1.0).sleep();

    }


    void Imu::deviceAck(const DeviceAck* ack){
        Device::deviceAck(ack);
        if(isReady()) {
            ros_utils::rosInfo("Imu is ready");
            _imuAMQ = _nodeHandle.advertise<sensor_msgs::Imu>("imu", 10);
            _imuM = _nodeHandle.advertise<sensor_msgs::MagneticField>("imu_M", 10);
        }
        else {
            ros_utils::rosError("RiCBoard can't build Imu object for spme reason, this program will shut down now");
            ros::shutdown();
        }
    }

    Imu::Imu(byte id, TransportLayer *transportLayer, uint16_t fusionHz, std::string frameId, bool enableGyro,
                 bool fuseCompass, std::vector<double> imuLinearAccFix, std::vector<double> imuAngularVelocityFix,
                 std::vector<double> imuMagnetometerFix, std::vector<double> imuRotationFix,
                 std::vector<double> imuRotationOffset)
            : Device(id, transportLayer) {
        _fusionHz = fusionHz;
        _frameId = frameId;
        _isStopClib = true;
        _isStateChange = false;
        _enableGyro = enableGyro;
        _fuseCompass = fuseCompass;
        _imuState = robotican_hardware_interface::setImuClibRequest::STOP;
        _setImuClibService = _nodeHandle.advertiseService("set_imu_calibration_state", &Imu::onSetImuClib, this);
        initArrays();

        _imuLinearAccFix[0][0] = imuLinearAccFix[0];
        _imuLinearAccFix[0][1] = imuLinearAccFix[1];
        _imuLinearAccFix[0][2] = imuLinearAccFix[2];
        _imuLinearAccFix[1][0] = imuLinearAccFix[3];
        _imuLinearAccFix[1][1] = imuLinearAccFix[4];
        _imuLinearAccFix[1][2] = imuLinearAccFix[5];
        _imuLinearAccFix[2][0] = imuLinearAccFix[6];
        _imuLinearAccFix[2][1] = imuLinearAccFix[7];
        _imuLinearAccFix[2][2] = imuLinearAccFix[8];

        _imuAngularVelocityFix[0][0] = imuAngularVelocityFix[0];
        _imuAngularVelocityFix[0][1] = imuAngularVelocityFix[1];
        _imuAngularVelocityFix[0][2] = imuAngularVelocityFix[2];
        _imuAngularVelocityFix[1][0] = imuAngularVelocityFix[3];
        _imuAngularVelocityFix[1][1] = imuAngularVelocityFix[4];
        _imuAngularVelocityFix[1][2] = imuAngularVelocityFix[5];
        _imuAngularVelocityFix[2][0] = imuAngularVelocityFix[6];
        _imuAngularVelocityFix[2][1] = imuAngularVelocityFix[7];
        _imuAngularVelocityFix[2][2] = imuAngularVelocityFix[8];

        _imuMagnetometerFix[0][0] = imuMagnetometerFix[0];
        _imuMagnetometerFix[0][1] = imuMagnetometerFix[1];
        _imuMagnetometerFix[0][2] = imuMagnetometerFix[2];
        _imuMagnetometerFix[1][0] = imuMagnetometerFix[3];
        _imuMagnetometerFix[1][1] = imuMagnetometerFix[4];
        _imuMagnetometerFix[1][2] = imuMagnetometerFix[5];
        _imuMagnetometerFix[2][0] = imuMagnetometerFix[6];
        _imuMagnetometerFix[2][1] = imuMagnetometerFix[7];
        _imuMagnetometerFix[2][2] = imuMagnetometerFix[8];

        _imuRotationFix[0][0] = imuRotationFix[0];
        _imuRotationFix[0][1] = imuRotationFix[1];
        _imuRotationFix[0][2] = imuRotationFix[2];
        _imuRotationFix[1][0] = imuRotationFix[3];
        _imuRotationFix[1][1] = imuRotationFix[4];
        _imuRotationFix[1][2] = imuRotationFix[5];
        _imuRotationFix[2][0] = imuRotationFix[6];
        _imuRotationFix[2][1] = imuRotationFix[7];
        _imuRotationFix[2][2] = imuRotationFix[8];

        _imuRotationOffset[0] = imuRotationOffset[0];
        _imuRotationOffset[1] = imuRotationOffset[1];
        _imuRotationOffset[2] = imuRotationOffset[2];



    }


    bool Imu::onSetImuClib(robotican_hardware_interface::setImuClib::Request &request, robotican_hardware_interface::setImuClib::Response &response) {
        if(request.state == robotican_hardware_interface::setImuClibRequest::CALIBRATION) {
            _isStateChange = _imuState != request.state;
            _isStopClib = false;
            _imuState = robotican_hardware_interface::setImuClibRequest::CALIBRATION;
            _clibPub = _nodeHandle.advertise<robotican_hardware_interface::imuClib>("imu_calibration", 10);
        }
        else if(request.state == robotican_hardware_interface::setImuClibRequest::STOP) {
            _isStateChange = _imuState != request.state;
            _isStopClib = true;
            _imuState = robotican_hardware_interface::setImuClibRequest::STOP;
            _clibPub.shutdown();
        }
        else if(request.state == robotican_hardware_interface::setImuClibRequest::SAVE) {
            _isStateChange = _imuState != request.state;
            _imuState = robotican_hardware_interface::setImuClibRequest::SAVE;
        }
        response.ack = true;
        return true;
    }

    void Imu::initArrays() {
        for(int i = 0; i < 3; ++i)
            for(int j = 0; j < 3; ++j) {
                _imuLinearAccFix[i][j] = 0.0;
                _imuAngularVelocityFix[i][j] = 0.0;
                _imuMagnetometerFix[i][j] = 0.0;
                _imuRotationFix[i][j] = 0.0;
            }
        _imuRotationOffset[0] =  _imuRotationOffset[1] = _imuRotationOffset[2] = 0.0;
    }
}
