//
// Created by tom on 17/04/16.
//

#ifndef ROBOTICAN_HARDWARE_INTERFACE_KOMODO_H
#define ROBOTICAN_HARDWARE_INTERFACE_KOMODO_H

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <dynamixel_msgs/JointState.h>
#include <robotican_hardware_interface/robot_base.h>
#include <robotican_hardware_interface/TorsoHomming.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/posvel_command_interface.h>
#include <robotican_hardware_interface/dynamixel_controller.h>

#define UP 1
#define DOWN -1

#define TORSO_DONT_MOVE 999
#define RAD_2_M 0.00175 / 2 * M_PI

namespace robotican_hardware {
    class KomodoRobot : public RobotBase {
    private:
        hardware_interface::PositionJointInterface _positionJointInterface;
        hardware_interface::PosVelJointInterface _posVelJointInterface;
        dynamixel_controller::DynamixelController* _dynamixelController;
        std::map<std::string, dynamixel_controller::JointInfo_t> _jointInfo;
        dynamixel_controller::JointInfo_t* _torsoJoint;

        ros::Subscriber _armStateListener;
        ros::Subscriber _upperSwitchListener;
        ros::Subscriber _lowerSwitchListener;
        ros::Publisher _armCmd;

        bool _first;
        double _lastTorsoRead;
        bool _reachHomingUpper;
        bool _reachHomingLower;
        bool _doneHome;

        bool buildDxlMotors();
        void armStateCallback(const sensor_msgs::JointStateConstPtr &msg);
        void onUpperSwitchClick(const std_msgs::BoolConstPtr &value);
        void onLowerSwitchClick(const std_msgs::BoolConstPtr &value);
        bool dxlPut2MapAndRegisterInterface(const std::string &jointName, const std::string &jointInterface);
    public:
        KomodoRobot();
        virtual ~KomodoRobot();
        virtual void registerInterfaces();
        virtual void read();
        virtual void write();

        bool loadHomeSwitch();
    };
}


#endif //ROBOTICAN_HARDWARE_INTERFACE_KOMODO_H
