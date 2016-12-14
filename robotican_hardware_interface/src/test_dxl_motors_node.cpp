//
// Created by tom on 14/12/16.
//

#include <ros/ros.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <robotican_hardware_interface/dynamixel_controller.h>

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

#define RAD_2_M 0.00175 / 2 * M_PI

class DxlTest : public hardware_interface::RobotHW {
private:
    ros::Time time_;
    bool first_;
    double torsoLastRead_;
    ros::Publisher armCmd_;
    ros::NodeHandle nodeHandle_;
    ros::Subscriber armStateListener_;
    hardware_interface::JointStateInterface jointStateInterface_;
    hardware_interface::VelocityJointInterface velocityJointInterface_;
    std::pair<std::string, dynamixel_controller::JointInfo_t> torsoInfo_;

    void armStateCallback(const sensor_msgs::JointStateConstPtr &msg) {
        dynamixel_controller::JointInfo_t &jointInfo = torsoInfo_.second;
        double currentPosition = msg->position[0];
            if(!first_) {
                double lastPosition = torsoLastRead_, positionDelta = (currentPosition - lastPosition);
                if (fabs(positionDelta) > 3.24) {
                    positionDelta = -sgn(positionDelta) * 2 * M_PI + positionDelta;
                }
//                ROS_INFO("[%s]: currentPosition = %.5f, lastPosition = %.5f, positionDeltaBefore = %.5f, positionDelta = %.5f ",
//                         ros::this_node::getName().c_str(), currentPosition, lastPosition, (currentPosition - lastPosition), positionDelta);
                jointInfo.position += (positionDelta * RAD_2_M);
            }
        torsoLastRead_ = currentPosition;
        first_ = false;
    }

public:

    DxlTest() {
        first_ = true;
        hardware_interface::JointStateHandle jointStateHandle("torso_joint", &torsoInfo_.second.position,
                                                              &torsoInfo_.second.velocity,
                                                              &torsoInfo_.second.effort);
        jointStateInterface_.registerHandle(jointStateHandle);

        hardware_interface::JointHandle jointHandle(jointStateInterface_.getHandle("torso_joint"), &torsoInfo_.second.cmd_vel);
        velocityJointInterface_.registerHandle(jointHandle);

        armStateListener_ = nodeHandle_.subscribe("dxl_joint_states", 10, &DxlTest::armStateCallback, this);
        armCmd_ = nodeHandle_.advertise<sensor_msgs::JointState>("joint_command", 10);
        registerInterface(&jointStateInterface_);
        registerInterface(&velocityJointInterface_);
    }

    ros::Time getTime() {
        return ros::Time::now();
    }

    ros::Duration getPeriod() {
        ros::Time now = getTime();
        ros::Duration period = now - time_;
        time_ = now;
        return period;;
    }

    void read() {

    }

    void write() {
        sensor_msgs::JointState jointCmd;
        jointCmd.name.push_back("torso_joint");
        jointCmd.position.push_back(torsoInfo_.second.cmd_pos);
        if(fabs(torsoInfo_.second.cmd_vel) > 12.2) torsoInfo_.second.cmd_vel = sgn(torsoInfo_.second.cmd_vel) * 12.2;
        jointCmd.velocity.push_back(-torsoInfo_.second.cmd_vel);
        jointCmd.effort.push_back(0.0);

        armCmd_.publish(jointCmd);
    }

};


int main(int argc, char** argv) {
    ros::init(argc, argv, "dxl_motor_test");
    ros::AsyncSpinner spinner(1);
    DxlTest robot;
    controller_manager::ControllerManager controllerManager(&robot);
    
    spinner.start();
    ros::Rate loopRate(100);
    while(ros::ok()) {
        robot.read();
        controllerManager.update(robot.getTime(), robot.getPeriod());
        robot.write();
        loopRate.sleep();
    }
    



    return 0;
}