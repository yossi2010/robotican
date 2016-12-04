//
// Created by tom on 07/04/16.
//

#ifndef ROBOTICAN_HARDWARE_INTERFACE_ARMADILLO_H
#define ROBOTICAN_HARDWARE_INTERFACE_ARMADILLO_H

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <dynamixel_msgs/JointState.h>
#include <robotican_hardware_interface/robot_base.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/posvel_command_interface.h>
#include <robotican_hardware_interface/dynamixel_controller.h>

namespace robotican_hardware {
    class ArmadilloRobot : public RobotBase {
    protected:
        hardware_interface::PositionJointInterface _positionJointInterface;
        hardware_interface::PosVelJointInterface _posVelJointInterface;

    private:
        bool _first;
        std::map<std::string, dynamixel_controller::JointInfo_t> _jointInfo;
        std::pair<std::string, JointInfo_t> _panInfo;
        std::pair<std::string, JointInfo_t> _tiltInfo;

        ros::Subscriber _armStateListener;
        ros::Publisher _armCmd;

        ros::Publisher _panCmd;
        ros::Publisher _tiltCmd;

        ros::Subscriber _panState;
        ros::Subscriber _tiltState;

        void panCallback(const dynamixel_msgs::JointState::ConstPtr &msg);
        void tiltCallback(const dynamixel_msgs::JointState::ConstPtr &msg);
        void armStateCallback(const sensor_msgs::JointStateConstPtr &msg);



    public:
        ArmadilloRobot();
        virtual ~ArmadilloRobot();
        virtual void registerInterfaces();
        virtual void read();
        virtual void write();
    };
}


#endif //ROBOTICAN_HARDWARE_INTERFACE_ARMADILLO_H
