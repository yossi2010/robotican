//
// Created by tom on 06/04/16.
//

#ifndef ROBOTICAN_HARDWARE_INTERFACE_ROBOT_BASE_H
#define ROBOTICAN_HARDWARE_INTERFACE_ROBOT_BASE_H

#include <ros/ros.h>
#include <ric_board/Motor.h>
#include <std_msgs/Float32.h>
#include <hardware_interface/robot_hw.h>
#include <robotican_hardware_interface/ros_utils.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <robotican_hardware_interface/Device.h>
#include <robotican_hardware_interface/RiCBoardManager.h>

namespace robotican_hardware {
    class Device;


    /*!
     * @brief Abstract class that all the robot will inherit from. This class will do the basic logic that apply to all the robots (for example init the RiCBoad cause alll the robots have one)
     */
    class RobotBase : public hardware_interface::RobotHW {
    private:
        ros::Time _time;
    protected:
        ros::NodeHandle _nodeHandle;
        hardware_interface::JointStateInterface _jointStateInterface;
        hardware_interface::VelocityJointInterface _velocityJointInterface;
        hardware_interface::ImuSensorInterface _imuSensorInterface;
#ifdef RIC_BOARD_TEST
        hardware_interface::PositionJointInterface _positionJointInterface;
#endif
        RiCBoardManager _boardManager;                                                                                  //!< The RiCBoard logic layer.

    public:
        RobotBase();
        virtual ~RobotBase();
        /*!
         * @brief Method that register all the hardware interfaces to the ros system.
         */
        virtual void registerInterfaces();
        /*!
         * @brief Do the read logic that needed to update the current joints state
         */
        virtual void read();
        /*!
         * @brief Do the write logic that needed to send the current joint command to all hardware
         */
        virtual void write();
        ros::Time getTime();
        ros::Duration getPeriod();
    };
}

#endif //ROBOTICAN_HARDWARE_INTERFACE_ROBOT_BASE_H
