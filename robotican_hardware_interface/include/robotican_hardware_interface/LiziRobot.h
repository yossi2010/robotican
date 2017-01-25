//
// Created by tom on 14/04/16.
//

#ifndef ROBOTICAN_HARDWARE_INTERFACE_LIZIROBOT_H
#define ROBOTICAN_HARDWARE_INTERFACE_LIZIROBOT_H

#include <ric_board/Motor.h>
#include <robotican_hardware_interface/robot_base.h>
#include <hardware_interface/joint_command_interface.h>
#include <robotican_hardware_interface/dynamixel_pro_controller.h>
#include <dynamixel_msgs/JointState.h>

namespace robotican_hardware {

    //typedef std::pair<std::string, JointInfo_t> joint_pair;

    class LiziRobot : public RobotBase {
    private:
   //     hardware_interface::PositionJointInterface _posJointInterface;

//        joint_pair _panInfo;
//        joint_pair _tiltInfo;
//        ros::Publisher _panPub;
//        ros::Publisher _tiltPub;
//        ros::Subscriber _panSub;
//        ros::Subscriber _tiltSub;
//        std::string _panSubTopic,
//                    _tiltSubTopic,
//                    _panPubTopic,
//                    _tiltPubTopic;

//        bool _havePanTilt;
//        void preparePanTilt();
//        void buildConnections();
//
//        void panCallBack(const dynamixel_msgs::JointState::ConstPtr &msg);
//        void tiltCallBack(const dynamixel_msgs::JointState::ConstPtr &msg);

    protected:

    public:
        LiziRobot();
        virtual ~LiziRobot();
        virtual void registerInterfaces();
        virtual void read();
        virtual void write();
    };
}

#endif //ROBOTICAN_HARDWARE_INTERFACE_LIZIROBOT_H
